#pragma once
#include "State.h"
#include "GNSS.h"

class Coast : public State {
	_STATE_CLASS_IMPLS_
	public:
		Coast(GNSS gnss);
	private:
	 GNSS gnss;
};
