#include "scheduler.h"
#include <string.h>

void scheduler_init(scheduler_t *sched)
{
    memset(sched, 0, sizeof(scheduler_t));
}

int scheduler_add_task(scheduler_t *sched, task_fn_t fn, uint32_t period_ticks)
{
    if (sched->n_tasks >= SCHED_MAX_TASKS) return -1;
    sched->tasks[sched->n_tasks].fn           = fn;
    sched->tasks[sched->n_tasks].period_ticks = period_ticks;
    sched->tasks[sched->n_tasks].last_tick    = 0;
    sched->tasks[sched->n_tasks].overrun_count = 0;
    sched->n_tasks++;
    return 0;
}

void scheduler_run(scheduler_t *sched, uint32_t now_ticks)
{
    for (uint8_t i = 0; i < sched->n_tasks; i++) {
        sched_task_t *t = &sched->tasks[i];
        uint32_t elapsed = now_ticks - t->last_tick;
        if (elapsed >= t->period_ticks) {
            if (elapsed > t->period_ticks + t->period_ticks)
                t->overrun_count++;
            t->last_tick = now_ticks;
            t->fn();
        }
    }
}

void scheduler_get_overruns(const scheduler_t *sched, uint32_t out[SCHED_MAX_TASKS])
{
    for (uint8_t i = 0; i < SCHED_MAX_TASKS; i++)
        out[i] = (i < sched->n_tasks) ? sched->tasks[i].overrun_count : 0;
}
