
#ifndef __UNOS_H_
#define __UNOS_H_

/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                          UNOS HEADER FILE                            */
/*                                                                      */
/*                                 by                                   */
/*                                                                      */
/*                             Robert Betz                              */
/*          Department of Electrical Engineering and Computer Science   */
/*                        University of Newcastle                       */
/*                                                                      */
/*                          ( Copyright 1989 )                          */
/*                                                                      */
/*                                                                      */
/************************************************************************/


/*
HISTORY

This file was created for version 1.5 of UNOS.

*/


/*
DESCRIPTION

This file contains the function definitions for the UNOS kernel file.  In
addition definitions used in the kernel module are defined.

*/

/************************************************************************/
/*                                                                      */
/*                          DATA STRUCTURE TYPES                        */
/*                                                                      */
/************************************************************************/



    /*------ timer structure type definition ------*/

    typedef
        struct time_struc {
            unsigned char status;
            unsigned char type;
            unsigned long init_time_cnt;
            unsigned long delta_time_cnt;
            struct time_struc* prev_timer_ptr;
            struct time_struc* next_timer_ptr;
            void ( *timer_handler )( void* );
            void* timer_handler_data;
        } timer_struc;



    /*------ task control block structure type definition ------*/

    typedef
        struct tsk_ctrl_blk {
            unsigned int task_num;
            char *task_name_ptr;
            unsigned char static_priority;
            unsigned char dynamic_priority;
            unsigned int ticks_before_kernel_entry;
            unsigned int ticks_to_go;
            unsigned char task_status;
            struct tsk_ctrl_blk* prev_task_ptr;
            struct tsk_ctrl_blk* next_task_ptr;
            struct time_struc* timer_ptr;
            char timeout_flag;
            char q_type;
            unsigned int cent_tab_index;
            unsigned int sema_blocked_on;
            unsigned int num_semas_claimed;
            unsigned int task_stkbase;
            unsigned int task_stkptr;
            unsigned int task_baseptr;
			unsigned char fp_save_area [ 150 ];
        } task_control_block;



    /*------ semaphore structure type definition ------*/

    typedef
        struct sema_struc {
            char *creating_taskname_ptr;
            unsigned int semaphore_value;
			unsigned int semaphore_multiplier;
			char semaphore_type;
            struct tsk_ctrl_blk* tcb_ptr;
            unsigned int semaph_bgn_q_index;
            void ( *semaph_queue_handler )();
        } semaphore_struc;







/************************************************************************/
/*                                                                      */
/*                      MAIL BOX RELATED DEFINITIONS                    */
/*                                                                      */
/************************************************************************/


    /*------ TASKNAME_MAP STRUCTURE DEFINITION ------*/
    /* this structure contains the connection between a task name and the
    task number associated with a task. The structure also contains a pointer
    to allow the structures to be placed in a linked list so that chained
    hashing can be implemented.
    */
    typedef
        struct taskname_struc {
            char *task_name_ptr;
            unsigned int task_number;
            struct taskname_struc *nxt_taskname_map;
        } taskname_map;



    /* Define the envelope structure */

    typedef
        struct {
            char *rtn_addr_ptr;
            unsigned int sender_pri;
            unsigned int mess_lgth;
            char* message_ptr;
        } envelope;


    /* Define the mail box structure */

    typedef
        struct {
            unsigned int mess_addr;
            char mbx_type;
            unsigned int q_size;
            unsigned int mess_size;
            unsigned int spce_avail_sema;
            unsigned int mess_avail_sema;
            unsigned int free;
            unsigned int used;
            unsigned int get_ptr;
            unsigned int put_ptr;
            char qik_mess_flag;
            envelope* qik_mess_ptr;
            envelope* mess_q_ptr;
        } mbx;




/************************************************************************/
/*                                                                      */
/*                        MEMORY ALLOCATION STRUCTURES                  */
/*                                                                      */
/************************************************************************/



/*------ HEADER STRUCTURE DEFINITION ------*/
/* The structure defined below is the header which is stored at the
beginning of every allocated block and every free block of storage. The
structure is defined as a union so that the header will be aligned according
to the most restrictive alignment type. The header structure contains a
pointer to the next free block and the size of the current free block.
The list of free blocks is arranged in order of memory address. The linked
list is circular in nature with the last free block being linked back to
the first free block.
*/

typedef
    int align;  /* forces alignment to int boundaries */

union header_struc {
    struct {
        union header_struc huge* nxt_blk_ptr;    /* pointer to next free block */
        unsigned long blk_size;             /* size of this free block */
    } header;
    align x;                                /* force alignment of blocks */
};

typedef
    union header_struc blk_header;




/************************************************************************/
/*                                                                      */
/*                       FUNCTION PROTOTYPE DECLARATIONS                */
/*                                                                      */
/************************************************************************/

extern void interrupt kernel ( void );

extern void interrupt tick ( void );

extern void wait ( unsigned int semaphore_num );

extern void _signal ( unsigned int semaphore_num );

extern unsigned int rtn_current_task_num ( void );

extern void reschedule ( void );

extern void stop_time_slice ( void );

extern void start_time_slice ( void );

extern int change_task_priority ( char *task_name_ptr,
                                unsigned char new_priority );

extern unsigned int rtn_task_priority ( char *task_name_ptr );

extern void chg_base_ticks_per_time_slice
        ( int new_base_ticks_per_time_slice );

extern int chg_task_tick_delta ( char *task_name_ptr, int new_tick_delta );

extern void start_tasks ( void ( *null_task ) ( void *) );

extern int create_task (  char *task_name_ptr, unsigned char priority_of_task,
                    int task_tick_delta, unsigned char status_of_task,
                    unsigned char q_type, unsigned int semaphore_num,
                    unsigned int task_stk_size, unsigned int mess_q_size,
                    unsigned int mess_size, void ( *init_task ) ( void ),
					void ( *task )( void* ), void* local_var_ptr );

extern void init_semaphore ( unsigned int sema_num, unsigned int sema_value,
                        unsigned int multiplier_value );

extern void preemptive_schedule ( void );

extern timer_struc* start_timer ( unsigned char timer_type,
                            unsigned long init_time_cnt,
							void ( *timeout_handler ) ( int * ),
                            void* data_ptr );

extern timer_struc* reset_timer ( timer_struc* timer_ptr );

extern timer_struc* stop_timer ( timer_struc* timer_ptr );

extern int timed_wait ( unsigned int sema_num, unsigned long timeout_value );

extern timer_struc* create_timer ( void );

extern int send_mess ( unsigned char* mess_ptr, unsigned int mess_lgth,
                char *mess_addr_ptr );

extern int send_qik_mess ( unsigned char* mess_ptr, unsigned int mess_lgth,
                    char * mess_addr_ptr );

extern char *rcv_mess ( unsigned char* mess_ptr, unsigned int* mess_lgth,
                        unsigned long time_limit );

extern unsigned int size_mbx ( char *mbx_addr_ptr );

extern unsigned int size_mbx_mess ( char *mbx_addr_ptr );

extern unsigned int free_mbx ( char *mbx_addr_ptr );

extern unsigned int used_mbx ( char *mbx_addr_ptr );

extern void flush_mbx ( void );

extern char setup_os_data_structures ( unsigned char kernel_interrupt_num,
                                int clock_ticks_kernel_ent,
                                unsigned int num_of_priorities,
                                unsigned int max_num_semaphores,
                                unsigned int maximum_num_of_tasks,
                                char* ptr_to_mem_pool,
                                unsigned long memory_pool_size );

extern char huge* umalloc ( unsigned long num_bytes );

extern void ufree ( char huge* blk_ptr );

extern char huge* ucalloc ( unsigned long num_obj, unsigned long size_of_obj );

extern unsigned long ret_free_mem ( void );

extern unsigned int return_semaphore_value ( unsigned int sema_num );

extern unsigned int create_semaphore ( void );

extern char *rtn_current_task_name_ptr ( void );

extern char return_start_flag (void);

extern void enable_task_switch (void);

extern void disable_task_switch (void);




/*------ Queue types used in the task creation routine ------*/

#define PRIORITY_Q_TYPE 0
#define SEMAPHORE_Q_TYPE 1
#define DONOT_Q_TYPE 2


/*------ Types of mail boxes ------*/

#define FIFO_TYPE 0
#define PRIORITY_TYPE 1




#endif