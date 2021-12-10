// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

#ifndef COMMUNICATIONS_ZMQ_SOCKET_BASE_H
#define COMMUNICATIONS_ZMQ_SOCKET_BASE_H

#include "ISocket.h"
#include "zmq.hpp"

#include <memory>

namespace uxas {
namespace communications {

// This class allows for different socket types to be created via constructor arguments.  For
// additional constructor/destructor setup should extend this class.

class ZmqSocketBase : public ISocket<const std::string&, bool> {
protected:
    // Initializer provides uncoupled method of instantiating socket.
    typedef std::shared_ptr<ISocket<std::shared_ptr<zmq::socket_t>&, const std::string&, int32_t, bool>>
        InitializerPtr;  

public:
    ZmqSocketBase() = default;
    ZmqSocketBase(InitializerPtr initializer, zmq::socket_type socketType) 
    : m_socketType{static_cast<int32_t>(socketType)}, m_initializer{initializer} {}

    virtual ~ZmqSocketBase() override {
        if (m_socket) {
            m_socket->setsockopt<uint32_t>(ZMQ_LINGER,0);
            m_socket->close();
        }
    };

    // Initialize the socket
    virtual bool initialize(const std::string& address, bool isServer) override {
        m_isServer = isServer;
        if (m_initializer->initialize(m_socket, address, m_socketType, m_isServer)) {
            m_routingId = std::move(m_socket->getsockopt<std::array<uint8_t,256>>(ZMQ_ROUTING_ID));
            return true;
        } else {
            return false;
        }
    }

    // Get pointer to the socket
    std::shared_ptr<zmq::socket_t> getSocket() { return m_socket; }

    // Return server status of this socket
    bool isServer() const { return m_isServer; }

    const std::array<uint8_t,256>& getRoutingId() const { return m_routingId; }

protected: 
    bool m_isServer{false};
    std::shared_ptr<zmq::socket_t> m_socket{nullptr};
    InitializerPtr m_initializer;
    int32_t m_socketType;
    std::array<uint8_t,256> m_routingId{};
};

}
}

#endif