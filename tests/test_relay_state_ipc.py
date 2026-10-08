import fcntl
import json
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from relay_state_ipc import StateChannel,CAPACITY,HEADER

class StateTests(unittest.TestCase):
    def test_latest_only_and_nonblocking_writer(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'state'
            writer=StateChannel(path,writer=True);reader=StateChannel(path)
            try:
                self.assertIsNone(reader.read())
                for i in range(10):self.assertTrue(writer.publish(dict(session='a',i=i)))
                self.assertEqual(reader.read()['i'],9)
                fcntl.flock(reader.file,fcntl.LOCK_SH)
                self.assertFalse(writer.publish(dict(session='a',i=10)))
                fcntl.flock(reader.file,fcntl.LOCK_UN)
                self.assertEqual(reader.read()['i'],9)
                self.assertTrue(writer.publish(dict(session='a',i=11)))
                self.assertEqual(reader.read()['i'],11)
            finally:reader.close();writer.close()

    def test_reader_skips_busy_writer_and_recovers_after_restart(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'state'
            writer=StateChannel(path,writer=True);reader=StateChannel(path)
            writer.publish(dict(session='first'))
            fcntl.flock(writer.file,fcntl.LOCK_EX)
            self.assertIsNone(reader.read())
            fcntl.flock(writer.file,fcntl.LOCK_UN)
            writer.close()
            writer=StateChannel(path,writer=True)
            writer.publish(dict(session='second'))
            self.assertEqual(reader.read()['session'],'second')
            reader.close();writer.close()

    def test_oversize_and_nonfinite_do_not_replace_valid_snapshot(self):
        with tempfile.TemporaryDirectory() as folder:
            writer=StateChannel(Path(folder)/'state',writer=True)
            writer.publish(dict(ok=True))
            with self.assertRaises(ValueError):writer.publish(dict(value='x'*CAPACITY))
            with self.assertRaises(ValueError):writer.publish(dict(value=float('nan')))
            self.assertTrue(writer.read()['ok'])
            writer.close()
