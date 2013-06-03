#ifndef MAPPROCESSING_MAP_EXCEPTION_H
#define MAPPROCESSING_MAP_EXCEPTION_H

#include <iostream>

namespace mapprocessing
{

class MapException : public std::exception
{
public:
	MapException(const char*) throw();
	virtual ~MapException() throw();
	virtual const char* what() const throw();
private:
	char *msg;
};

}

#endif
