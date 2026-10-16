#ifndef CORE_SCHEDULER_H
#define CORE_SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

#define SCHED_MAX_TASKS 8U

typedef void (*task_fn_t)(void);

typedef struct {
    task_fn_t fn;
    uint32_t  period_ticks;
    uint32_t  last_tick;
    uint32_t  overrun_count;
} sched_task_t;

typedef struct {
    sched_task_t tasks[SCHED_MAX_TASKS];
    uint8_t      n_tasks;
} scheduler_t;

void scheduler_init(scheduler_t *sched);
int  scheduler_add_task(scheduler_t *sched, task_fn_t fn, uint32_t period_ticks);
void scheduler_run(scheduler_t *sched, uint32_t now_ticks);
void scheduler_get_overruns(const scheduler_t *sched, uint32_t out[SCHED_MAX_TASKS]);

#endif
