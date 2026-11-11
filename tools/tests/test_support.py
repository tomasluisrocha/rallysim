import os
import unittest
from unittest.mock import patch

from .support import need


class TestNeed(unittest.TestCase):
    def test_false_condition_fails_in_ci(self):
        test = unittest.TestCase()
        with patch.dict(os.environ, {"RALLYTOOLS_CI": "1"}):
            with self.assertRaises(test.failureException):
                need(test, False, "required")

    def test_false_condition_skips_without_ci(self):
        test = unittest.TestCase()
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaises(unittest.SkipTest):
                need(test, False, "required")

    def test_true_condition_returns(self):
        test = unittest.TestCase()
        with patch.dict(os.environ, {"RALLYTOOLS_CI": "1"}):
            self.assertIsNone(need(test, True, "required"))


if __name__ == "__main__":
    unittest.main()
