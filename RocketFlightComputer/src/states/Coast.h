#pragma once
#include "State.h"

#define MAX_COAST_TIME 30000
class Coast : public State {
	_STATE_CLASS_IMPLS_
	public:
	Coast();
	private:
	boolean apogeePassed = false;
};
