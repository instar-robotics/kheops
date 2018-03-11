#include "fexample.h"

REGISTER_FUNCTION(FExample);

void FExample::compute()
{
	std::cout << "FExample : " << getUuid() << std::endl;
}

void  FExample::setparameters()
{
	Kernel::instance().bind(is,"is", getUuid());
	Kernel::instance().bind(ism,"ism", getUuid());
	Kernel::instance().bind(str,"str", getUuid());
	Kernel::instance().bind(isanc,"isanc", getUuid());
	Kernel::instance().bind(ismanc,"ismanc", getUuid());
	Kernel::instance().bind(immanc,"immanc", getUuid());
}

