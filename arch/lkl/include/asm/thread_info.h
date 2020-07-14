#ifndef _ASM_LKL_THREAD_INFO_H
#define _ASM_LKL_THREAD_INFO_H

#define THREAD_SIZE	       (4096)
#define PREEMPT_ACTIVE (0x10000000)

#ifndef __ASSEMBLY__
#include <asm/types.h>
#include <asm/processor.h>
#include <asm/host_ops.h>

typedef struct {
	unsigned long seg;
} mm_segment_t;

struct thread_info {
	struct task_struct *task;
	struct exec_domain	*exec_domain;
	unsigned long flags;
	int preempt_count;
	mm_segment_t addr_limit;
	struct lkl_sem *sched_sem;
	bool dead;
	lkl_thread_t tid;
	struct task_struct *prev_sched;
	unsigned long stackend;

	struct restart_block	restart_block;
};

#define INIT_THREAD_INFO(tsk)				\
{							\
	.task		= &tsk,				\
	.preempt_count	= INIT_PREEMPT_COUNT,		\
	.exec_domain    = &default_exec_domain,         \
	.flags		= 0,				\
	.addr_limit	= KERNEL_DS,			\
	.restart_block = {		  	        \
		.fn = do_no_restart_syscall,	        \
	},	                                        \
}

#define init_thread_info	(init_thread_union.thread_info)
#define init_stack		(init_thread_union.stack)

/* how to get the thread information struct from C */
extern struct thread_info *_current_thread_info;
static inline struct thread_info *current_thread_info(void)
{
	return _current_thread_info;
}

/* thread information allocation */
struct thread_info *alloc_thread_info_node(struct task_struct *, int node);
void free_thread_info(struct thread_info *ti);

int threads_init(void);
void threads_cleanup(void);

#define TIF_SYSCALL_TRACE		0
#define TIF_NOTIFY_RESUME		1
#define TIF_SIGPENDING			2
#define TIF_NEED_RESCHED		3
#define TIF_RESTORE_SIGMASK		4
#define TIF_MEMDIE			5
#define TIF_NOHZ			6

#define __HAVE_THREAD_FUNCTIONS

#define task_thread_info(task)	((struct thread_info *)(task)->stack)
#define task_stack_page(task)	((task)->stack)
void setup_thread_stack(struct task_struct *p, struct task_struct *org);
#define end_of_stack(p) (&task_thread_info(p)->stackend)

#endif /* __ASSEMBLY__ */

#endif
