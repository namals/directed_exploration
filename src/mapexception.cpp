#include <directed_exploration/mapprocessing/mapexception.h>

#include <cstring>

using namespace mapprocessing;

MapException::MapException(const char* str) throw()
{
	msg = new char[strlen(str) + 1];
	strcpy(msg, str);
}

MapException::~MapException() throw()
{
	delete msg;
}

const char* MapException::what() const throw()
{
	return msg;
}
