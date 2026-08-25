#include "nevo_ws_bridge/ws_client.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

namespace nevo_ws_bridge
{

WsClient::WsClient()
{
}

WsClient::~WsClient()
{
  stop();
}

void WsClient::configure(
  const std::string & primary_uri,
  const std::string & backup_uri,
  const std::string & robot_id,
  double reconnect_interval_sec)
{
  primary_uri_ = primary_uri;
  backup_uri_ = backup_uri;
  robot_id_ = robot_id;
  reconnect_interval_sec_ = reconnect_interval_sec;
}

void WsClient::setMessageCallback(MessageCallback cb)
{
  message_cb_ = std::move(cb);
}

void WsClient::setStatusCallback(StatusCallback cb)
{
  status_cb_ = std::move(cb);
}

void WsClient::start()
{
  if (primary_uri_.empty()) {
    std::cerr << "[WsClient] primary_uri is empty, not starting WS\n";
    return;
  }

  use_backup_ = false;
  reconnecting_ = false;
  session_acknowledged_ = false;
  duplicate_retry_active_ = false;

  // Set URL awal ke primary
  ws_.setUrl(primary_uri_);

  // Callback dari ixwebsocket
  ws_.setOnMessageCallback(
    [this](const ix::WebSocketMessagePtr & msg)
    {
      switch (msg->type) {
        case ix::WebSocketMessageType::Open:
        {
          if (use_backup_) {
            std::cout << "[WsClient] Connected to BACKUP: " << backup_uri_ << std::endl;
            notifyStatus(ConnectionState::FALLBACK);
          } else {
            std::cout << "[WsClient] Connected to PRIMARY: " << primary_uri_ << std::endl;
            notifyStatus(ConnectionState::CONNECTED);
          }

          reconnecting_.store(false);

          session_acknowledged_ = false;
          sendSi();
          break;
        }

        case ix::WebSocketMessageType::Message:
          {
            std::string msg_str = msg->str;
            if (looksLikeDuplicate(msg_str)) {
              handleDuplicateSession();
            } else {
              // First non-error reply after Open means the cloud accepted our
              // session (SI success). Once acknowledged, the UI commands are
              // actually routed to THIS stack.
              if (!session_acknowledged_.load()) {
                session_acknowledged_ = true;
                std::cout << "[WsClient] Session acknowledged by cloud (robot_id="
                          << robot_id_ << ")" << std::endl;
              }
              if (message_cb_) {
                message_cb_(msg_str);
              }
            }
          }
          break;

        case ix::WebSocketMessageType::Close:
          std::cout << "[WsClient] Connection closed. Reason: "
                    << msg->closeInfo.reason << std::endl;
          notifyStatus(ConnectionState::DISCONNECTED);
          scheduleReconnect(true);
          break;

        case ix::WebSocketMessageType::Error:
          std::cout << "[WsClient] Error: " << msg->errorInfo.reason << std::endl;
          notifyStatus(ConnectionState::DISCONNECTED);
          scheduleReconnect(true);
          break;

        default:
          break;
      }
    });

  notifyStatus(ConnectionState::CONNECTING);
  ws_.start();
}

void WsClient::stop()
{
  ws_.stop();
  notifyStatus(ConnectionState::DISCONNECTED);
}

void WsClient::send(const std::string & msg)
{
  auto s = state_.load();
  if (s != ConnectionState::CONNECTED && s != ConnectionState::FALLBACK) {
    std::cerr << "[WsClient] Not connected, cannot send\n";
    return;
  }
  ws_.sendText(msg);
}

ConnectionState WsClient::getState() const
{
  return state_.load();
}

void WsClient::notifyStatus(ConnectionState s)
{
  state_.store(s);
  if (status_cb_) {
    status_cb_(s);
  }
}

void WsClient::sendSi()
{
  std::string si = std::string("{\"code\":\"SI\",\"data\":{\"type\":\"ROBOT\",\"robot_id\":\"")
    + robot_id_ + "\"}}";
  send(si);
  std::cout << "[WsClient] Sent SI: " << si << std::endl;
}

bool WsClient::looksLikeDuplicate(const std::string & payload)
{
  return payload.find("DUPLICATE_ROBOT_ID") != std::string::npos;
}

// The cloud holds the previous (old-stack) session for a while after it
// disconnects. While it holds it, our SI is rejected with DUPLICATE_ROBOT_ID
// and every UI command is routed to the dead/old session -> the swap hangs.
// Strategy: bounded SI retries to ride out the old session's expiry window,
// then a fresh full reconnect (a new TCP+SI handshake) once the window passes.
void WsClient::handleDuplicateSession()
{
  if (duplicate_retry_active_.exchange(true)) {
    return;  // one recovery sequence already running
  }

  std::cout << "[WsClient] DUPLICATE_ROBOT_ID detected, starting bounded "
            << "session recovery (robot_id=" << robot_id_ << ")" << std::endl;

  std::thread([this]() {
    for (int delay : {5, 10, 15}) {
      std::this_thread::sleep_for(std::chrono::seconds(delay));
      if (session_acknowledged_.load()) {
        std::cout << "[WsClient] Session acknowledged, stopping recovery" << std::endl;
        duplicate_retry_active_ = false;
        return;
      }
      ConnectionState s = state_.load();
      if (s != ConnectionState::CONNECTED && s != ConnectionState::FALLBACK) {
        // Socket dropped; normal reconnect machinery will take over.
        duplicate_retry_active_ = false;
        return;
      }
      sendSi();
      std::cout << "[WsClient] Retried SI after " << delay << "s" << std::endl;
    }

    // ~30s elapsed and the cloud still will not accept this session: the old
    // session should have expired by now, so drop and reconnect fresh.
    std::this_thread::sleep_for(std::chrono::seconds(5));
    if (!session_acknowledged_.load()) {
      std::cout << "[WsClient] Session still rejected after retries, "
                << "forcing fresh reconnect" << std::endl;
      forceReconnect();
    }
    duplicate_retry_active_ = false;
  }).detach();
}

void WsClient::forceReconnect()
{
  if (reconnecting_.exchange(true)) {
    return;
  }
  ws_.stop();
  use_backup_ = false;
  session_acknowledged_ = false;
  ws_.setUrl(primary_uri_);
  notifyStatus(ConnectionState::CONNECTING);
  ws_.start();
}

// 🔹 Fallback / reconnect logic utama
void WsClient::scheduleReconnect(bool from_error)
{
  if (reconnecting_.exchange(true)) {
    // sudah ada thread reconnect yang jalan
    return;
  }

  std::thread([this]() {
    std::this_thread::sleep_for(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(reconnect_interval_sec_)));

    // Tentukan target berikutnya:
    // - Kalau sekarang pakai primary & ada backup → pindah ke backup (FALLBACK)
    // - Kalau sekarang pakai backup → balik ke primary
    bool next_use_backup = use_backup_.load();

    if (!next_use_backup && !backup_uri_.empty()) {
      next_use_backup = true;
      std::cout << "[WsClient] Fallback: trying BACKUP WS: " << backup_uri_ << std::endl;
      notifyStatus(ConnectionState::FALLBACK);
    } else {
      next_use_backup = false;
      std::cout << "[WsClient] Reconnect: trying PRIMARY WS: " << primary_uri_ << std::endl;
      notifyStatus(ConnectionState::CONNECTING);
    }

    use_backup_.store(next_use_backup);
    session_acknowledged_ = false;
    ws_.stop();
    ws_.setUrl(next_use_backup ? backup_uri_ : primary_uri_);
    ws_.start();
  }).detach();
}

}  // namespace nevo_ws_bridge
