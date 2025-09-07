// Chapter 3
// The C language allows the thread_info structure and the kernel stack of a
// process to be conveniently represented by means of the following union
// construct:
union thread_union {
  struct thread_info thread_info;
  unsigned long stack[2048]; /* 1024 for 4KB stacks */
};

// Another useful macro, called for_each_process, scans the whole process list.
// It is defined as:
#define for_each_process(p)                                                 \
  for (p = &init_task; (p = list_entry((p)->tasks.next, struct task_struct, \
                                       tasks)) != &init_task;)

//  The enqueue_task(p,array) function inserts a process descriptor into a
//  runqueue list;
its code is essentially equivalent to
    : list_add_tail(&p->run_list, &array->queue[p->prio]);
__set_bit(p->prio, array->bitmap);
array->nr_active++;
p->array = array;

// The pidhash_shift variable stores the length in bits of a table index (11, in
// our example). The hash_long() function is used by many hash functions; on a
// 32-bit architecture it is essentially equivalent to:
unsigned long hash_long(unsigned long val, unsigned int bits) {
  unsigned long hash = val * 0x9e370001UL;
  return hash >> (32 - bits);
}

// Wait queues are implemented as doubly linked lists whose elements include
// pointers to process descriptors. Each wait queue is identified by a wait
// queue head, a data structure of type wait_queue_head_t:
struct __wait_queue_head {
  spinlock_t lock;
  struct list_head task_list;
};
typedef struct __wait_queue_head wait_queue_head_t;

// Synchronization is achieved by the lock spin lock in the wait queue head. The
// task_list field is the head of the list of waiting processes. Elements of a
// wait queue list are of type wait_queue_t:
struct __wait_queue {
  unsigned int flags;
  struct task_struct *task;
  wait_queue_func_t func;
  struct list_head task_list;
};
typedef struct __wait_queue wait_queue_t;

// The init_waitqueue_entry(q,p) function initializes a wait_queue_t structure q
// as follows:
q->flags = 0;
q->task = p;
q->func = default_wake_function;

// The sleep_on() function operates on the current process:
void sleep_on(wait_queue_head_t *wq) {
  wait_queue_t wait;
  init_waitqueue_entry(&wait, current);
  current->state = TASK_UNINTERRUPTIBLE;
  add_wait_queue(wq, &wait); /* wq points to the wait queue head */
  schedule();
  remove_wait_queue(wq, &wait);
}

// The prepare_to_wait(), prepare_to_wait_exclusive(), and finish_wait()
// functions, introduced in Linux 2.6, offer yet another way to put the current
// process to sleep in a wait queue. Typically, they are used as follows:
DEFINE_WAIT(wait);
prepare_to_wait_exclusive(&wq, &wait, TASK_INTERRUPTIBLE);
/* wq is the head of the wait queue */
... if (!condition) schedule();
finish_wait(&wq, &wait);

// The wait_event and wait_event_interruptible macros put the calling process to
// sleep on a wait queue until a given condition is verified. For instance, the
// wait_ event(wq,condition) macro essentially yields the following fragment:
DEFINE_WAIT(_ _wait);
for (;;) {
  prepare_to_wait(&wq, &_ _wait, TASK_UNINTERRUPTIBLE);
  if (condition) break;
  schedule();
}
finish_wait(&wq, &_ _wait);

// the wake_up macro is essentially equivalent to the following code fragment:
void wake_up(wait_queue_head_t *q) {
  struct list_head *tmp;
  wait_queue_t *curr;
  list_for_each(tmp, &q->task_list) {
    curr = list_entry(tmp, wait_queue_t, task_list);
    if (curr->func(curr, TASK_INTERRUPTIBLE | TASK_UNINTERRUPTIBLE, 0, NULL) &&
        curr->flags)
      break;
  }
}

// The _ _switch_
// to( ) function is declared in the include/asm-i386/system.h header file as
// follows:
_ _switch_to(struct task_struct *prev_p,
             struct task_struct *next_p) _ _attribute_ _(regparm(3));

//  These registers need not be saved, because the prev_p->thread.debugreg array
//  is modified only when a debugger wants to monitor prev:
if (next_p->thread.debugreg[7]) {
  loaddebug(&next_p->thread, 0);
  loaddebug(&next_p->thread, 1);
  loaddebug(&next_p->thread, 2);
  loaddebug(&next_p->thread, 3);
  /* no 4 and 5 */
  loaddebug(&next_p->thread, 6);
  loaddebug(&next_p->thread, 7);
}

// They are stored in the thread.i387 subfield of the process descriptor, whose
// format is described by the i387_union union:
union i387_union {
  struct i387_fsave_struct fsave;
  struct i387_fxsave_struct fxsave;
  struct i387_soft_struct soft;
};

// the first time the next process tries to execute an ESCAPE, MMX, or SSE/SSE2
// instruction, the control unit raises a “Device not available” exception, and
// the kernel (more precisely, the exception handler involved by the exception)
// runs the math_state_restore( ) function. The next process is identified by
// this handler as current.
void math_state_restore() {
  asm volatile("clts"); /* clear the TS flag of cr0 */
  if (!(current->flags & PF_USED_MATH)) init_fpu(current);
  restore_fpu(current);
  current->thread.status |= TS_USEDFPU;
}

// chapter 4
set_trap_gate(0,&divide_error);
set_trap_gate(1,&debug);
set_intr_gate(2,&nmi);
set_system_intr_gate(3,&int3);
set_system_gate(4,&overflow);
set_system_gate(5,&bounds);
set_trap_gate(6,&invalid_op);
set_trap_gate(7,&device_not_available);
set_task_gate(8,31);
set_trap_gate(9,&coprocessor_segment_overrun);
set_trap_gate(10,&invalid_TSS);
set_trap_gate(11,&segment_not_present);
set_trap_gate(12,&stack_segment);
set_trap_gate(13,&general_protection);
set_intr_gate(14,&page_fault);
set_trap_gate(16,&coprocessor_error);
set_trap_gate(17,&alignment_check);
set_trap_gate(18,&machine_check);
set_trap_gate(19,&simd_coprocessor_error);
set_system_gate(128,&system_call);
do_IRQ()
irq_exit()
smp_apic_timer_interrupt()
in_interrupt()
the _ _do_softirq()
local_irq_restore()
local_softirq_pending())
local_irq_enable()
local_irq_disable()
wakeup_softirqd()
local_irq_restore()
TASKLET_SOFTIRQ()
create_workqueue("foo")
schedule_work(w) 
queue_work(keventd_wq,w)
schedule_delayed_work(w,d)
 queue_delayed_work(keventd_wq,w,d) 
schedule_delayed_work_on(cpu,w,d)
 queue_delayed_work(keventd_wq,w,d) 
flush_scheduled_work() 
flush_workqueue(keventd_wq)
ret_from_exception()
ret_from_intr()

// chapter 5

// The read_lock macro, applied to the address rwlp of a read/write spin lock,
// is similar to the spin_lock macro described in the previous section. If the
// kernel preemption option has been selected when the kernel was compiled, the
// macro performs the very same actions as those of spin_lock(), with just one
// exception: to effectively acquire the read/write spin lock in step 2, the
// macro executes the _raw_read_trylock() function:
int _raw_read_trylock(rwlock_t *lock) {
  atomic_t *count = (atomic_t *)lock->lock;
  atomic_dec(count);
  if (atomic_read(count) >= 0) return 1;
  atomic_inc(count);
  return 0;
}

//  The write_lock macro is implemented in the same way as spin_lock() and read_
//  lock(). For instance, if kernel preemption is supported, the function
//  disables kernel preemption and tries to grab the lock right away by invoking
//  _raw_write_trylock(). If this function returns 0, the lock was already
//  taken, thus the macro reenables kernel preemption and starts a busy wait
//  loop, as explained in the description of spin_ lock() in the previous
//  section. The _raw_write_trylock() function is shown below:
int _raw_write_trylock(rwlock_t *lock) {
  atomic_t *count = (atomic_t *)lock->lock;
  if (atomic_sub_and_test(0x01000000, count)) return 1;
  atomic_add(0x01000000, count);
  return 0;
}

// A seqlock_t variable is initialized to “unlocked” either by assigning to it
// the value SEQLOCK_UNLOCKED, or by executing the seqlock_init macro. Writers
// acquire and release a seqlock by invoking write_seqlock() and
// write_sequnlock(). The first function acquires the spin lock in the seqlock_t
// data structure, then increases the sequence counter by one; the second
// function increases the sequence counter once more, then releases the spin
// lock. This ensures that when the writer is in the middle of writing, the
// counter is odd, and that when no writer is altering data, the counter is
// even. Readers implement a critical region as follows:
unsigned int seq;
do {
  seq = read_seqbegin(&seqlock);
  /* ... CRITICAL REGION ... */
} while (read_seqretry(&seqlock, seq));

// Let’s start by discussing how to release a semaphore, which is much simpler
// than getting one. When a process wishes to release a kernel semaphore lock,
// it invokes theup() function, where _ _up() is the following C function:
__attribute__((regparm(3))) void __up(struct semaphore *sem) {
  wake_up(&sem->wait);
}

// when a process wishes to acquire a kernel semaphore lock, it invokes the
// down( ) function. The implementation of down( ) is quite involved,where _
// _down() is the following C function:
__attribute__((regparm(3))) void __down(struct semaphore *sem) {
  DECLARE_WAITQUEUE(wait, current);
  unsigned long flags;
  current->state = TASK_UNINTERRUPTIBLE;
  spin_lock_irqsave(&sem->wait.lock, flags);
  add_wait_queue_exclusive_locked(&sem->wait, &wait);
  sem->sleepers++;
  for (;;) {
    if (!atomic_add_negative(sem->sleepers - 1, &sem->count)) {
      sem->sleepers = 0;
      break;
    }
    sem->sleepers = 1;
    spin_unlock_irqrestore(&sem->wait.lock, flags);
    schedule();
    spin_lock_irqsave(&sem->wait.lock, flags);
    current->state = TASK_UNINTERRUPTIBLE;
  }
  remove_wait_queue_locked(&sem->wait, &wait);
  wake_up_locked(&sem->wait);
  spin_unlock_irqrestore(&sem->wait.lock, flags);
  current->state = TASK_RUNNING;
}

// The completion is a synchronization primitive that is specifically designed
// to solve this problem. The completion data structure includes a wait queue
// head and a flag:
struct completion {
  unsigned int done;
  wait_queue_head_t wait;
};

// The lock_kernel( ) and unlock_kernel( ) functions are used to get and release
// the big kernel lock. The former function is equivalent to:
depth = current->lock_depth + 1;
if (depth == 0) down(&kernel_sem);
current->lock_depth = depth;

// while the latter is equivalent to:
if (--current->lock_depth < 0) up(&kernel_sem);


// chapter 9
// Check if the memory region identified by mmap_cache includes the given address.
// If yes, return the region descriptor pointer.
vma = mm->mmap_cache;
if (vma && vma->vm_end > addr && vma->vm_start <= addr)
    return vma;

// Otherwise, scan the memory regions of the process and look up the memory region in the red-black tree.
rb_node = mm->mm_rb.rb_node; // Start from the root of the red-black tree
vma = NULL;                  // Initialize the pointer to the memory region descriptor

// Traverse the red-black tree to find the memory region containing the address
while (rb_node)
{
    vma_tmp = rb_entry(rb_node, struct vm_area_struct, vm_rb); // Get the memory region from the tree node
    if (vma_tmp->vm_end > addr)
    {                                  // If the end address of the region is greater than the given address
        vma = vma_tmp;                 // Update the pointer to the memory region
        if (vma_tmp->vm_start <= addr) // If the start address of the region is less than or equal to the given address
            break;                     // Break the loop as the region containing the address is found
        rb_node = rb_node->rb_left;    // Move to the left child as the address is within the current region
    }
    else
    {
        rb_node = rb_node->rb_right; // Move to the right child as the address is beyond the current region
    }
}

// If a memory region containing the address is found, update the mmap_cache to optimize future lookups.
if (vma)
    mm->mmap_cache = vma;

return vma; // Return the pointer to the memory region containing the address, if found

// Find the memory region containing 'start_addr'.
vma = find_vma(mm, start_addr);

// If 'end_addr' precedes the start of the found region, set 'vma' to NULL.
if (vma && end_addr <= vma->vm_start)
    vma = NULL;

// Return the pointer to the found region or NULL if not suitable.
return vma;

// Check if 'len' exceeds the maximum task size
if (len > TASK_SIZE)
    return -ENOMEM;

// Align 'addr' to a page boundary
addr = (addr + 0xfff) & 0xfffff000;

// If 'addr' and 'addr + len' fit within the task size, try to find a suitable region
if (addr && addr + len <= TASK_SIZE)
{
    vma = find_vma(current->mm, addr);
    if (!vma || addr + len <= vma->vm_start)
        return addr;
}

// Initialize 'start_addr' with 'addr'
start_addr = addr;

// Iterate over memory regions to find a suitable address range
for (vma = find_vma(current->mm, addr);; vma = vma->vm_next)
{
    // If 'addr + len' exceeds the task size, return an error
    if (addr + len > TASK_SIZE)
    {
        if (start_addr == (TASK_SIZE / 3 + 0xfff) & 0xfffff000)
            return -ENOMEM;
        start_addr = addr = (TASK_SIZE / 3 + 0xfff) & 0xfffff000;
        vma = find_vma(current->mm, addr);
    }
    // If no overlapping region found, update 'free_area_cache' and return 'addr'
    if (!vma || addr + len <= vma->vm_start)
    {
        mm->free_area_cache = addr + len;
        return addr;
    }
    addr = vma->vm_end;
}

// Set info.si_code to indicate a segmentation violation due to mapping error
info.si_code = SEGV_MAPERR;

// If the address is beyond the task size
if (address >= TASK_SIZE)
{
    // If the error code doesn't indicate a read or write access
    if (!(error_code & 0x101))
        goto vmalloc_fault; // Handle the fault in vmalloc space
    else
        goto bad_area_nosemaphore; // Handle the fault in non-vmalloc space
}

// Try to acquire a read lock on the mmap semaphore
if (!down_read_trylock(&tsk->mm->mmap_sem))
{
    // If the error code doesn't indicate a write access and the address is not in the exception table
    if ((error_code & 4) == 0 && !search_exception_table(regs->eip))
        goto bad_area_nosemaphore; // Handle the fault without semaphore
    else
        down_read(&tsk->mm->mmap_sem); // Acquire the read lock
}

// Find the memory region containing the address
vma = find_vma(tsk->mm, address);

// If no memory region is found, handle the fault
if (!vma)
    goto bad_area;

// If the address is within the memory region, continue
if (vma->vm_start <= address)
    goto good_area;

// If the memory region doesn't allow stack growth downwards, handle the fault
if (!(vma->vm_flags & VM_GROWSDOWN))
    goto bad_area;

// If the fault occurred in user mode and the address is beyond the stack limit, handle the fault
if (error_code & 4 && address + 32 < regs->esp)
    goto bad_area;

// Attempt to expand the stack, if unsuccessful, handle the fault
if (expand_stack(vma, address))
    goto bad_area;

// If no issues encountered, proceed
goto good_area;

// If the fault occurred in user mode
if (error_code & 4)
{
    // Set the CR2 register to the faulting address
    tsk->thread.cr2 = address;
    // Set the error code with the address flag indicating if it's above the TASK_SIZE limit
    tsk->thread.error_code = error_code | (address >= TASK_SIZE);
    // Set the trap number to 14 (Page Fault)
    tsk->thread.trap_no = 14;
    // Set up signal info for SIGSEGV
    info.si_signo = SIGSEGV;
    info.si_errno = 0;
    info.si_addr = (void *)address;
    // Send SIGSEGV signal to the task
    force_sig_info(SIGSEGV, &info, tsk);
    // Return, indicating the signal has been sent
    return;
}

// Set info.si_code to indicate a segmentation violation due to access error
info.si_code = SEGV_ACCERR;

// Initialize write flag
write = 0;

// Check if it's a write access
if (error_code & 2)
{ /* write access */
    // If the memory region doesn't allow write access, handle the fault
    if (!(vma->vm_flags & VM_WRITE))
        goto bad_area;
    // Set the write flag
    write++;
}
else
{ /* read access */
    // If it's a read access with error or the memory region doesn't allow read or execute access, handle the fault
    if ((error_code & 1) || !(vma->vm_flags & (VM_READ | VM_EXEC)))
        goto bad_area;
}

// If the fault is due to a bus error
if (ret == VM_FAULT_SIGBUS)
{
// Handle bus error
do_sigbus:
    // Release the mmap semaphore
    up_read(&tsk->mm->mmap_sem);
    // If it's a kernel mode fault, go to no_context
    if (!(error_code & 4)) /* Kernel Mode */
        goto no_context;
    // Set up signal info for SIGBUS
    tsk->thread.cr2 = address;
    tsk->thread.error_code = error_code;
    tsk->thread.trap_no = 14;
    info.si_signo = SIGBUS;
    info.si_errno = 0;
    // Set signal code to indicate address error
    info.si_code = BUS_ADRERR;
    info.si_addr = (void *)address;
    // Send SIGBUS signal to the task
    force_sig_info(SIGBUS, &info, tsk);
}

// Get the page directory entry (PGD) corresponding to the address
pgd = pgd_offset(mm, address);

// Acquire the page table lock to prevent concurrent access
spin_lock(&mm->page_table_lock);

// Allocate a page upper directory (PUD) for the address if possible
pud = pud_alloc(mm, pgd, address);
if (pud)
{
    // Allocate a page middle directory (PMD) for the address if possible
    pmd = pmd_alloc(mm, pud, address);
    if (pmd)
    {
        // Allocate and map a page table entry (PTE) for the address if possible
        pte = pte_alloc_map(mm, pmd, address);
        if (pte)
            // Handle the page table entry fault and return the result
            return handle_pte_fault(mm, vma, address, write_access, pte, pmd);
    }
}

// Release the page table lock
spin_unlock(&mm->page_table_lock);

// Return out-of-memory fault code if allocation fails
return VM_FAULT_OOM;

// If the VMA has no page operations or no-page handler, handle it as anonymous page
if (!vma->vm_ops || !vma->vm_ops->nopage)
    return do_anonymous_page(mm, vma, page_table, pmd, write_access, address);

// Handle write access
if (write_access)
{
    // Unmap the page table to avoid concurrent access
    pte_unmap(page_table);
    // Allocate a new page with zeroed contents
    page = alloc_page(GFP_HIGHUSER | __GFP_ZERO);
    // Remap the page table
    page_table = pte_offset_map(pmd, addr);
    // Increment the resident set size (RSS)
    mm->rss++;
    // Create a new page table entry with dirty bit set and maybe make it writable
    entry = maybe_mkwrite(pte_mkdirty(mk_pte(page, vma->vm_page_prot)), vma);
    // Add the page to the active least recently used (LRU) cache
    lru_cache_add_active(page);
    // Set the page as referenced
    SetPageReferenced(page);
    // Set the new page table entry
    set_pte(page_table, entry);
    // Unmap the page table
    pte_unmap(page_table);
    // Return minor fault code
    return VM_FAULT_MINOR;
}

// Handle vmalloc fault
vmalloc_fault :
    // Get the physical address of the page global directory (PGD)
    asm("movl %%cr3,%0" : "=r"(pgd_paddr));
// Calculate the virtual address of the page global directory (PGD)
pgd = pgd_index(address) + (pgd_t *)__va(pgd_paddr);
// Get the kernel page global directory (PGD)
pgd_k = init_mm.pgd + pgd_index(address);
// Check if the kernel PGD entry is present
if (!pgd_present(*pgd_k))
    goto no_context;
// Offset to the page upper directory (PUD)
pud = pud_offset(pgd, address);
pud_k = pud_offset(pgd_k, address);
// Check if the PUD entry is present
if (!pud_present(*pud_k))
    goto no_context;
// Offset to the page middle directory (PMD)
pmd = pmd_offset(pud, address);
pmd_k = pmd_offset(pud_k, address);
// Check if the PMD entry is present
if (!pmd_present(*pmd_k))
    goto no_context;
// Set the PMD entry
set_pmd(pmd, *pmd_k);
// Offset to the page table entry (PTE)
pte_k = pte_offset_kernel(pmd_k, address);
// Check if the PTE entry is present
if (!pte_present(*pte_k))
    goto no_context;
// Return if everything is fine
return;

// If CLONE_VM flag is set, share the memory space with the parent process
if (clone_flags & CLONE_VM)
{
    // Increment the reference count of the current process's memory descriptor
    atomic_inc(&current->mm->mm_users);
    // Wait for the page table lock to be released
    spin_unlock_wait(&current->mm->page_table_lock);
    // Set the new process's memory descriptor to the parent's memory descriptor
    tsk->mm = current->mm;
    tsk->active_mm = current->mm;
    // Return success
    return 0;
}

// Otherwise, create a new memory descriptor for the child process
tsk->mm = kmem_cache_alloc(mm_cachep, SLAB_KERNEL);
// Copy parent's memory descriptor to the child's
memcpy(tsk->mm, current->mm, sizeof(*tsk->mm));
// Set the reference count and user count to 1
atomic_set(&tsk->mm->mm_users, 1);
atomic_set(&tsk->mm->mm_count, 1);
// Initialize the mmap semaphore and other fields
init_rwsem(&tsk->mm->mmap_sem);
tsk->mm->core_waiters = 0;
tsk->mm->page_table_lock = SPIN_LOCK_UNLOCKED;
tsk->mm->ioctx_list_lock = RW_LOCK_UNLOCKED;
tsk->mm->ioctx_list = NULL;
tsk->mm->default_kioctx = INIT_KIOCTX(tsk->mm->default_kioctx, *tsk->mm);
// Set the free area cache and allocate a new page global directory (PGD)
tsk->mm->free_area_cache = (TASK_SIZE / 3 + 0xfff) & 0xfffff000;
tsk->mm->pgd = pgd_alloc(tsk->mm);
// Set default flags to 0
tsk->mm->def_flags = 0;
// chapter 10

//  A validity check is then performed on the system call number passed by the
//  User Mode process. If it is greater than or equal to the number of entries
//  in the system call dispatch table, the system call handler terminates:
cmpl $NR_syscalls, % eax jb nobadsys movl $(-ENOSYS),
    24(% esp) jmp resume_userspace nobadsys :

    // The check on addresses passed to system calls is performed by the
    // access_ok( ) macro, which acts on two parameters: addr and size. The
    // macro checks the address interval delimited by addr and addr + size – 1.
    // It is essentially equivalent to the following C function:
    int access_ok(const void *addr, unsigned long size) {
  unsigned long a = (unsigned long)addr;
  if (a + size < a || a + size > current_thread_info()->addr_limit.seg)
    return 0;
  return 1;
}

//  Each macro requires exactly 2 + 2 × n parameters, with n being the number of
//  parameters of the system call. The first two parameters specify the return
//  type and the name of the system call; each additional pair of parameters
//  specifies the type and the name of the corresponding system call parameter.
//  Thus, for instance, the wrapper routine of the fork( ) system call may be
//  generated by:
// _syscall0(int,fork)
//  while the wrapper routine of the write( ) system call may be generated by:
//  _syscall3(int,write,int,fd,const char *,buf,unsigned int,count) In the
//  latter case, the macro yields the following code:
int write(int fd, const char *buf, unsigned int count) {
  long __res;
  asm("int $0x80"
      : "=a"(__res)
      : "0"(__NR_write), "b"((long)fd), "c"((long)buf), "d"((long)count));
  if ((unsigned long)__res >= (unsigned long)-129) {
    errno = -__res;
    __res = -1;
  }
  return (int)__res;
}
// chapter 13
// According to this technique, the CPU checks (polls) the device’s status
// register repeatedly until its value signals that the I/O operation has been
// completed. We have already encountered a technique based on polling in the
// section “Spin Locks” in Chapter 5: when a processor tries to acquire a busy
// spin lock, it repeatedly polls the variable until its value becomes 0.
// However, polling applied to I/O operations is usually more elaborate, because
// the driver must also remember to check for possible time-outs. A simple
// example of polling looks like the following:
for (;;) {
  if (read_status(device) & DEVICE_END_OPERATION) break;
  if (--count == 0) break;
}

// The foo_read() function is triggered whenever the user reads the device file:
ssize_t foo_read(struct file *filp, char *buf, size_t count, loff_t *ppos) {
  foo_dev_t *foo_dev = filp->private_data;
 if (down_interruptible(&foo_dev->sem)
 return -ERESTARTSYS;
 foo_dev->intr = 0;
 outb(DEV_FOO_READ, DEV_FOO_CONTROL_PORT);
 wait_event_interruptible(foo_dev->wait, (foo_dev->intr= =1));
 if (put_user(foo_dev->data, buf))
 return -EFAULT;
 up(&foo_dev->sem);
 return 1;
}

// f the foo_interrupt() function:
irqreturn_t foo_interrupt(int irq, void *dev_id, struct pt_regs *regs) {
  foo->data = inb(DEV_FOO_DATA_PORT);
  foo->intr = 1;
  wake_up_interruptible(&foo->wait);
  return 1;
}
