# ─────────────────────────────────────────────────────────────────────────────
# One service pass, and the loop around it:
#   wait for records → read → fold closed epochs → commit the words →
#   drain commands → confirm arms → re-arm noise references → heartbeat.
# ─────────────────────────────────────────────────────────────────────────────

"""
    service_pass!(core; wait_ms = 1)

One pass of the loop process's service loop. Blocks in the driver's wait for at
most `wait_ms` when nothing is pending, and never otherwise.
"""
function service_pass!(core::LoopCore; wait_ms::Integer = 1)
    t0 = time_ns()
    wait_records(core.driver, wait_ms)
    taken = take_records!(core)
    now_reference = sample_count(core.driver, 1)
    if taken > 0 && core.latest_sample_index != typemin(Int64)
        age_us = round(Int64, (now_reference - core.latest_sample_index) * 1e6 / core.bands[1].entry.sampling_freq_hz)
        age_us = max(age_us, Int64(0))
        @inbounds core.latency_hist[_latency_bin(age_us)] += 1
        age_us > core.max_record_age_us && (core.max_record_age_us = age_us)
    end
    fold_closed_epochs!(core, now_reference)
    commit_words!(core)
    handle_commands!(core)
    confirm_arms!(core)
    rearm_noise_references!(core)
    loop_heartbeat!(core.segment)
    elapsed = Int64(time_ns() - t0)
    elapsed > core.max_pass_ns && (core.max_pass_ns = elapsed)
    nothing
end

"""
    run!(core; max_wait_ms = 1)

The service loop: passes until a shutdown command arrives (or `core.running`
is cleared). Marks the loop running in the segment's header on entry and
stopped on exit.
"""
function run!(core::LoopCore; max_wait_ms::Integer = 1)
    set_loop_state!(core.segment, HardwareLoopProtocol.LOOP_STATE_RUNNING)
    loop_heartbeat!(core.segment)
    while core.running
        service_pass!(core; wait_ms = max_wait_ms)
    end
    set_loop_state!(core.segment, HardwareLoopProtocol.LOOP_STATE_STOPPED)
    nothing
end
