// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou
 *
 * This file is a collections of functions and structures from gptimer_priv.h, gptimer.c and timer_hal.c
 *
 * They have been updated to be marked as forced inline, in order to be used from ISR in IRAM.
 * This is required to be able to use ISR in IRAM, while doing some flash operations.
 */
#pragma once

#include <driver/gptimer.h>
#include <esp_check.h>
#include <esp_err.h>
#include <esp_memory_utils.h>
#include <esp_pm.h>
#include <freertos/FreeRTOS.h>
#include <hal/timer_hal.h>
#include <hal/timer_ll.h>
#include <rom/ets_sys.h>
#include <stdatomic.h>
#include <sys/lock.h>

///////////////////////////////////////////////////////////////////////////
// FROM
///////////////////////////////////////////////////////////////////////////

typedef struct gptimer_group_t {
    int group_id;
    portMUX_TYPE spinlock; // to protect per-group register level concurrent access
    gptimer_t* timers[SOC_TIMER_GROUP_TIMERS_PER_GROUP];
#if GPTIMER_USE_RETENTION_LINK
    sleep_retention_module_t sleep_retention_module; // sleep retention module
    bool retention_link_created;                     // mark if the retention link is created
#endif
} gptimer_group_t;

typedef enum {
  GPTIMER_FSM_INIT,        // Timer is initialized, but not enabled
  GPTIMER_FSM_ENABLE,      // Timer is enabled, but is not running
  GPTIMER_FSM_ENABLE_WAIT, // Timer is in the middle of the enable process (Intermediate state)
  GPTIMER_FSM_RUN,         // Timer is in running
  GPTIMER_FSM_RUN_WAIT,    // Timer is in the middle of the run process (Intermediate state)
} gptimer_fsm_t;

struct gptimer_t {
    gptimer_group_t* group;
    int timer_id;
    uint32_t resolution_hz;
    uint64_t reload_count;
    uint64_t alarm_count;
    gptimer_count_direction_t direction;
    timer_hal_context_t hal;
    std::atomic<gptimer_fsm_t> fsm;
    int intr_priority;
    intr_handle_t intr;
    portMUX_TYPE spinlock; // to protect per-timer resources concurrent accessed by task and ISR handler
    gptimer_alarm_cb_t on_alarm;
    void* user_ctx;
    gptimer_clock_source_t clk_src;
    esp_pm_lock_handle_t pm_lock; // power management lock
#if CONFIG_PM_ENABLE
    char pm_lock_name[GPTIMER_PM_LOCK_NAME_LEN_MAX]; // pm lock name
#endif
    struct {
        uint32_t intr_shared : 1;
        uint32_t auto_reload_on_alarm : 1;
        uint32_t alarm_en : 1;
    } flags;
};

__attribute__((always_inline)) inline esp_err_t inlined_gptimer_get_raw_count(gptimer_handle_t timer, unsigned long long* value) {
  ESP_RETURN_ON_FALSE_ISR(timer && value, ESP_ERR_INVALID_ARG, "inlined_gptimer", "invalid argument");

  portENTER_CRITICAL_SAFE(&timer->spinlock);
  timer_ll_trigger_soft_capture((&timer->hal)->dev, (&timer->hal)->timer_id);
  *value = timer_ll_get_counter_value((&timer->hal)->dev, (&timer->hal)->timer_id);
  portEXIT_CRITICAL_SAFE(&timer->spinlock);
  return ESP_OK;
}

__attribute__((always_inline)) inline esp_err_t inlined_gptimer_set_raw_count(gptimer_handle_t timer, unsigned long long value) {
  ESP_RETURN_ON_FALSE_ISR(timer, ESP_ERR_INVALID_ARG, "inlined_gptimer", "invalid argument");

  portENTER_CRITICAL_SAFE(&timer->spinlock);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // - `timer_ll_set_reload_value()` will only indicate the `reload_value`
  // - `timer_ll_set_reload_value()` + ``timer_ll_trigger_soft_reload()` can update the HW counter value by software
  // Therefore, after updating the HW counter value, we need to restore the previous `reload_value`.
  // Attention: The following process should be protected by a lock in the driver layer.
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // save current reload value
  uint64_t old_reload = timer_ll_get_reload_value((&timer->hal)->dev, (&timer->hal)->timer_id);
  timer_ll_set_reload_value((&timer->hal)->dev, (&timer->hal)->timer_id, value);
  timer_ll_trigger_soft_reload((&timer->hal)->dev, (&timer->hal)->timer_id);
  // restore the previous reload value
  timer_ll_set_reload_value((&timer->hal)->dev, (&timer->hal)->timer_id, old_reload);
  portEXIT_CRITICAL_SAFE(&timer->spinlock);
  return ESP_OK;
}

__attribute__((always_inline)) inline esp_err_t inlined_gptimer_start(gptimer_handle_t timer) {
  ESP_RETURN_ON_FALSE_ISR(timer, ESP_ERR_INVALID_ARG, "inlined_gptimer", "invalid argument");

  gptimer_fsm_t expected_fsm = GPTIMER_FSM_ENABLE;
  if (atomic_compare_exchange_strong(&timer->fsm, &expected_fsm, GPTIMER_FSM_RUN_WAIT)) {
    // the register used by the following LL functions are shared with other API,
    // which is possible to run along with this function, so we need to protect
    portENTER_CRITICAL_SAFE(&timer->spinlock);
    timer_ll_enable_alarm(timer->hal.dev, timer->timer_id, timer->flags.alarm_en);
    timer_ll_enable_counter(timer->hal.dev, timer->timer_id, true);
    portEXIT_CRITICAL_SAFE(&timer->spinlock);
  } else {
    return ESP_ERR_INVALID_STATE;
  }

  atomic_store(&timer->fsm, GPTIMER_FSM_RUN);
  return ESP_OK;
}

__attribute__((always_inline)) inline esp_err_t inlined_gptimer_stop(gptimer_handle_t timer) {
  ESP_RETURN_ON_FALSE_ISR(timer, ESP_ERR_INVALID_ARG, "inlined_gptimer", "invalid argument");

  gptimer_fsm_t expected_fsm = GPTIMER_FSM_RUN;
  if (atomic_compare_exchange_strong(&timer->fsm, &expected_fsm, GPTIMER_FSM_ENABLE_WAIT)) {
    // disable counter, alarm, auto-reload
    portENTER_CRITICAL_SAFE(&timer->spinlock);
    timer_ll_enable_counter(timer->hal.dev, timer->timer_id, false);
    timer_ll_enable_alarm(timer->hal.dev, timer->timer_id, false);
    portEXIT_CRITICAL_SAFE(&timer->spinlock);
  } else {
    return ESP_ERR_INVALID_STATE;
  }
  atomic_store(&timer->fsm, GPTIMER_FSM_ENABLE);
  return ESP_OK;
}

__attribute__((always_inline)) inline esp_err_t inlined_gptimer_set_alarm_action(gptimer_handle_t timer, const gptimer_alarm_config_t* config) {
  ESP_RETURN_ON_FALSE_ISR(timer, ESP_ERR_INVALID_ARG, "inlined_gptimer", "invalid argument");

  if (config) {
#if CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM
    ESP_RETURN_ON_FALSE_ISR(esp_ptr_internal(config), ESP_ERR_INVALID_ARG, "inlined_gptimer", "alarm config struct not in internal RAM");
#endif
    // When auto_reload is enabled, alarm_count should not be equal to reload_count
    bool valid_auto_reload = !config->flags.auto_reload_on_alarm || config->alarm_count != config->reload_count;
    ESP_RETURN_ON_FALSE_ISR(valid_auto_reload, ESP_ERR_INVALID_ARG, "inlined_gptimer", "reload count can't equal to alarm count");

    portENTER_CRITICAL_SAFE(&timer->spinlock);
    timer->reload_count = config->reload_count;
    timer->alarm_count = config->alarm_count;
    timer->flags.auto_reload_on_alarm = config->flags.auto_reload_on_alarm;
    timer->flags.alarm_en = true;

    timer_ll_set_reload_value(timer->hal.dev, timer->timer_id, config->reload_count);
    timer_ll_set_alarm_value(timer->hal.dev, timer->timer_id, config->alarm_count);
    portEXIT_CRITICAL_SAFE(&timer->spinlock);

  } else {
    portENTER_CRITICAL_SAFE(&timer->spinlock);
    timer->flags.auto_reload_on_alarm = false;
    timer->flags.alarm_en = false;
    portEXIT_CRITICAL_SAFE(&timer->spinlock);
  }

  portENTER_CRITICAL_SAFE(&timer->spinlock);
  timer_ll_enable_auto_reload(timer->hal.dev, timer->timer_id, timer->flags.auto_reload_on_alarm);
  timer_ll_enable_alarm(timer->hal.dev, timer->timer_id, timer->flags.alarm_en);
  portEXIT_CRITICAL_SAFE(&timer->spinlock);
  return ESP_OK;
}
