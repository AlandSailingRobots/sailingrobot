#ifndef __THREADRAII_H__
#define __THREADRAII_H__

#include <thread>

class ThreadRAII {
public:
	enum class DtorAction { join, detach };

	ThreadRAII(std::thread&& t, DtorAction a);

	~ThreadRAII();
	std::thread& get();

private: 
	DtorAction m_action;
	std::thread m_t;

};
#endif