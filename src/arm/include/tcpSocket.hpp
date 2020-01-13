#ifndef __TCPSOCKET_HPP__
#define __TCPSOCKET_HPP__

#include <iostream> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 

class tcpSocket {
	public:
		tcpSocket(const std::string &ip, int port);
		~tcpSocket() { disconnect(); }
		bool send(const std::string &msg);
		std::string read();
    	bool connect();
    	bool available();
    protected:
        void warn(const std::string &msg) const;
		bool disconnect();
	private:
		std::string m_ip;
		int m_port;
		int m_socket;
};

#endif