#include "ThreadRAII.h"



	
ThreadRAII::ThreadRAII(std::thread&& t, DtorAction a)
: m_action(a), m_t(std::move(t)){}

ThreadRAII::~ThreadRAII(){
	if(m_t.joinable()){
		if(m_action == DtorAction::join){
			m_t.join();
		}
		else{
			m_t.detach();
		}
	}
}

std::thread& ThreadRAII::get() { 
	return m_t; 
}



