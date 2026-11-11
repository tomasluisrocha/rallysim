import os


def need(test, condition, why):
    if condition:
        return
    if os.environ.get("RALLYTOOLS_CI") == "1":
        test.fail(why)
    test.skipTest(why)
