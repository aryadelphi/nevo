#include "nevo_ws_bridge/ws_server.hpp"
#include <iostream>
#include <sstream>
#include <chrono>

WsServer::WsServer(int port)
    : port_(port),
      server_(port, "0.0.0.0")
{
}

WsServer::~WsServer()
{
    stop();
}

void WsServer::start()
{
    running_ = true;

    // 🔥 WAJIB supaya tidak error extension
    server_.disablePerMessageDeflate();

    server_.setOnClientMessageCallback(
        [this](std::shared_ptr<ix::ConnectionState>,
               ix::WebSocket& webSocket,
               const ix::WebSocketMessagePtr& msg)
        {
            if (msg->type == ix::WebSocketMessageType::Open)
            {
                {
                    std::lock_guard<std::mutex> lock(clients_mutex_);
                    clients_.insert(&webSocket);
                }

                std::cout << "[LOCAL CONNECTED]\n";

                webSocket.send(R"({
                    "code":"CONNECTED",
                    "data":{"message":"IX Local Server Ready"}
                })");

            }

            else if (msg->type == ix::WebSocketMessageType::Close)
            {
                {
                    std::lock_guard<std::mutex> lock(clients_mutex_);
                    clients_.erase(&webSocket);
                }

                std::cout << "[LOCAL DISCONNECTED] Reason: "
                          << msg->closeInfo.reason << "\n";
            }

            else if (msg->type == ix::WebSocketMessageType::Message)
            {
                if (message_callback_)
                    message_callback_(msg->str);
            }
        });

    
    // open local websocket port and bind to 0.0.0.0:8765 from constructor
    auto res = server_.listen();
    if (!res.first)
    {
        std::cerr << "Local WS listen error: "
                  << res.second << std::endl;
        return;
    }

    // activate local websocket
    server_.start();

    std::cout << "Local IX WebSocket running on port "
              << port_ << std::endl;

}

void WsServer::stop()
{
    running_ = false;

    if (battery_thread_.joinable())
        battery_thread_.join();

    server_.stop();
}


void WsServer::setMessageCallback(MessageCallback cb)
{
    message_callback_ = cb;
}

void WsServer::sendToAll(const std::string& message)
{
    std::lock_guard<std::mutex> lock(clients_mutex_);

    for (auto* client : clients_)
    {
        if (client)
            client->send(message);
    }
}




