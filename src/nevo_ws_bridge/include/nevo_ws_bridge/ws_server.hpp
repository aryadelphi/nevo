#pragma once

#include <ixwebsocket/IXWebSocketServer.h>
#include <functional>
#include <set>
#include <mutex>
#include <thread>
#include <atomic>
#include <string>

class WsServer
{
public:
    using MessageCallback = std::function<void(const std::string&)>;

    WsServer(int port);
    ~WsServer();

    void start();
    void stop();

    void setMessageCallback(MessageCallback cb);
    void sendToAll(const std::string& message);

private:
    int port_;
    ix::WebSocketServer server_;

    std::set<ix::WebSocket*> clients_;
    std::mutex clients_mutex_;

    std::thread battery_thread_;
    std::atomic<bool> running_{false};

    MessageCallback message_callback_;
};
