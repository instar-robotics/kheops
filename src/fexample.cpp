#include "fexample.h"

REGISTER_FUNCTION(Example);

void Example::compute()
{
	std::cout << "FExample : " << getUuid() << std::endl;
}

void  Example::setparameters()
{

	u_is.setMultiple(false);
	m_is.setMultiple(true);
	
	u_im.setMultiple(false);
	m_im.setMultiple(true);

	u_ism.setMultiple(false);
	m_ism.setMultiple(true);
	
	u_imm.setMultiple(false);
	m_imm.setMultiple(true);

	Kernel::instance().bind(str,"str", getUuid());
	Kernel::instance().bind(u_is,"u_is", getUuid());
	Kernel::instance().bind(m_is,"m_is", getUuid());
	Kernel::instance().bind(u_ism,"u_ism", getUuid());
	Kernel::instance().bind(m_ism,"m_ism", getUuid());
	Kernel::instance().bind(u_imm,"u_imm", getUuid());
	Kernel::instance().bind(m_imm,"m_imm", getUuid());
	Kernel::instance().bind(u_im,"u_im", getUuid());
	Kernel::instance().bind(m_im,"u_im", getUuid());
}

