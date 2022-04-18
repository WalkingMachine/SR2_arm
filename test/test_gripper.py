#   Copyright (c) 2022 Walking Machine
#
#   Permission is hereby granted, free of charge, to any person obtaining a copy of
#   this software and associated documentation files (the "Software"), to deal in
#   the Software without restriction, including without limitation the rights to
#   use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
#   the Software, and to permit persons to whom the Software is furnished to do so,
#   subject to the following conditions:
#
#   The above copyright notice and this permission notice shall be included in all
#   copies or substantial portions of the Software.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
#   FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
#   COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
#   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
#   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

__author__ = "Walking Machine"
__copyright__ = "Copyright (C) 2022 Walking Machine"
__license__ = "MIT"
__version__ = "1.0"

import pytest

from sr2_arm.robotiq_85.robotiq_test_gripper import RobotiqTestGripper


@pytest.fixture()
def mock_gripper():
    return RobotiqTestGripper()


@pytest.mark.mock_gripper
def test_mock_shutdown(mock_gripper):
    assert mock_gripper.shutdown()


@pytest.mark.mock_gripper
def test_mock_processes(mock_gripper):
    assert mock_gripper.process_act_cmd()
    assert mock_gripper.process_stat_cmd()
