#pragma once

// Creates and starts the serial consumer task (priority 1).
// This is the only place in the firmware that maps SystemEventID_t to
// human-readable strings. Must be called after system_queues_init().
void serial_consumer_start(void);

// Standard formatted log output for Core 0 tasks:
// Produces "@ <sec>.<dec> <TAG> : <key>=<val> ..."
// Subject to g_log_filter via log_filter_is_enabled(tag).
void serial_log(const char *tag, const char *fmt, ...);
