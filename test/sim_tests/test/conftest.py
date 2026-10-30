def pytest_terminal_summary(terminalreporter, exitstatus, config):
    loop_summary = getattr(config, "loop_summary", None)
    crash_summary = getattr(config, "crash_summary", None)
    if loop_summary:
        terminalreporter.write_sep("=", "Loop summary")
        terminalreporter.write_line(loop_summary)
    if(crash_summary):
        terminalreporter.write_sep("=", "Crash summary")
        terminalreporter.write_line(crash_summary)