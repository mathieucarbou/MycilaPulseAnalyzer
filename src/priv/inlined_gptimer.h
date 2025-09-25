// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2025 Mathieu Carbou
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
#include <sys/lock.h>

#include <atomic>

///////////////////////////////////////////////////////////////////////////
// FROM gptimer_priv.h
///////////////////////////////////////////////////////////////////////////

typedef struct gptimer_group_t {
    int group_id;
    portMUX_TYPE spinlock; // to protect per-group register level concurrent access
    gptimer_t* timers[SOC_TIMER_GROUP_TIMERS_PER_GROUP];
} gptimer_group_t;

typedef enum {
  GPTIMER_FSM_INIT,   // Timer is initialized, but not enabled yet
  GPTIMER_FSM_ENABLE, // Timer is enabled, but is not running yet
  GPTIMER_FSM_RUN,    // Timer is in running
  GPTIMER_FSM_WAIT,   // Timer is in the middle of state change (Intermediate state)
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

///////////////////////////////////////////////////////////////////////////
// FROM gptimer.c
///////////////////////////////////////////////////////////////////////////

__attribute__((always_inline)) inline esp_err_t inlined_gptimer_get_raw_count(gptimer_handle_t timer, unsigned long long* value) {
  if (timer == NULL || value == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  portENTER_CRITICAL_SAFE(&timer->spinlock);
  timer_ll_trigger_soft_capture((&timer->hal)->dev, (&timer->hal)->timer_id);
  *value = timer_ll_get_counter_value((&timer->hal)->dev, (&timer->hal)->timer_id);
  portEXIT_CRITICAL_SAFE(&timer->spinlock);
  return ESP_OK;
}

__attribute__((always_inline)) inline esp_err_t inlined_gptimer_set_raw_count(gptimer_handle_t timer, unsigned long long value) {
  if (timer == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
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

__attribute__((always_inline)) inline esp_err_t inlined_gptimer_set_alarm_action(gptimer_handle_t timer, const gptimer_alarm_config_t* config) {
  if (timer == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (config) {
#if CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM
    // when the function is placed in IRAM, we expect the config struct is also placed in internal RAM
    // if the cache is disabled, the function can still access the config struct
    if (esp_ptr_internal(config) == false) {
      return ESP_ERR_INVALID_ARG;
    }
#endif
    // When auto_reload is enabled, alarm_count should not be equal to reload_count
    bool valid_auto_reload = !config->flags.auto_reload_on_alarm || config->alarm_count != config->reload_count;
    if (valid_auto_reload == false) {
      return ESP_ERR_INVALID_ARG;
    }

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
