import pytest
import asyncio
from .test_suite_manager import TestSuiteManager

@pytest.mark.asyncio
async def test_full_suite():
    """
    This test will instantiate the TestSuiteManager and run the full test suite.
    This acts as the main entry point for pytest to run our testing framework.
    """
    print("--- Starting test run via pytest ---")
    manager = TestSuiteManager()
    results = await manager.run_full_test_suite()

    # Assert that all tests passed
    passed_all = all(r.passed for r in results)

    if not passed_all:
        failed_tests = [f"{r.test_name}: {r.error_message}" for r in results if not r.passed]
        pytest.fail(f"One or more tests failed: {'; '.join(failed_tests)}", pytrace=False)

    print("--- All tests in the suite passed successfully. ---")
