def pytest_terminal_summary(terminalreporter, exitstatus, config):
    summary = getattr(config, "loop_summary", None)
    if summary:
        terminalreporter.write_sep("=", "Loop summary")
        terminalreporter.write_line(summary)