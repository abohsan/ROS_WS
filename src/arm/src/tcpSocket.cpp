#include "../include/tcpSocket.hpp"

tcpSocket::tcpSocket(const std::string &ip, int port){
    m_ip = ip;
    m_port = port;
    m_socket = 0;
}
bool tcpSocket::connect() {
    struct sockaddr_in serv_addr; 
    if ((m_socket = ::socket(AF_INET, SOCK_STREAM, 0)) < 0) { 
        warn("creation error"); 
        return false; 
    } 
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(m_port); 
    if (::inet_pton(AF_INET, m_ip.c_str(), &serv_addr.sin_addr) <= 0) { 
        warn("invalid address, address not supported"); 
        return false; 
    } 
    if (::connect(m_socket, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) { 
        warn("connection failed"); 
        return false; 
    } 
  	return true;
}

bool tcpSocket::disconnect() {
	if ( ::shutdown(m_socket,2) < 0 ) {
		warn("problem closing socket");
		return false;
	}
	return true;
}
bool tcpSocket::available() {

	
	return true;
}

bool tcpSocket::send(const std::string &msg) {
	if (::send(m_socket , msg.c_str() , msg.size() , 0) < 0) {
		warn("send message faild");
		return false;
	} 
	::fsync(m_socket);
	return true;
}

std::string tcpSocket::read() {
	static char buffer[512];
	if (::read( m_socket , buffer, 512) < 0) {
		warn("read message failed");
		return std::string();
	}
	return std::string(buffer);
}

void tcpSocket::warn(const std::string &msg) const {
	std::cout << "Ops, socket: " << msg << "!" 
	          << std::endl; 
}
