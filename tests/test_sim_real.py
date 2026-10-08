import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from compare_sim_real import compare, summarize


class ComparisonTests(unittest.TestCase):
    def test_missing_measurements_are_not_zero(self):
        result = compare([{'passed': True, 'duration_s': 10}], [{'passed': False}])
        self.assertEqual(result['real_minus_sim']['success_rate'], -1)
        self.assertIsNone(result['real_minus_sim']['duration_s'])
        self.assertEqual(result['real']['duration_s']['measured_trials'], 0)

    def test_invalid_or_unmeasured_trials_rejected(self):
        for reports in ([], [{'passed': None}], [{'passed': True, 'duration_s': float('nan')}], [{'passed': True, 'duration_s': -1}]):
            with self.assertRaises(ValueError):
                summarize(reports)
