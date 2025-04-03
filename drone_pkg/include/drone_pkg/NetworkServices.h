class NetworkServices
{
public:
	static int sendMessage(int curSocket, const char* message, int messageSize);
	static int receiveMessage(int curSocket, char* buffer, int bufSize);
};



