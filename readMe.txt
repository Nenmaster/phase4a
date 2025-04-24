// TestCase 0 , TestCase 1, TestCase 2 
The differences in output for test00 through test03 are due to expected variations in timing caused by how the system clock and interrupts work. since the sleep() system call uses microsecond timing and interrupt handling, exact start and end times can vary slightly between runs. 
