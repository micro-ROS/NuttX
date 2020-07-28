#include <ctf_common.h>

inline int irq_lock(void)
{
	return enter_critical_section();
}

inline void irq_unlock(int key)
{
	leave_critical_section(key);
}

inline bool z_is_idle_thread_object(struct tcb_s *thread)
{
	return (0 == thread->pid);
}
