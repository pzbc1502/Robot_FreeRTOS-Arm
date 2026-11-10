"""Execute the actual stack hook with fake GPIO and a non-returning CPU halt."""
import test_scan_workflow as workflow
from test_scan_motion import function
from test_stop_confirmation import run_c


def main():
    hook = function((workflow.ROOT/'src/robot_thread_entry.c').read_text(), 'vApplicationStackOverflowHook')
    run_c(r'''
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
typedef void *TaskHandle_t;
static jmp_buf halted;
static unsigned sequence;
static void BSP_Laser_Off(void){assert(sequence==0);sequence=1;}
static void disable_interrupts(void){assert(sequence==1);sequence=2;}
static void halt_cpu(void){assert(sequence==2);longjmp(halted,1);}
#define taskDISABLE_INTERRUPTS() disable_interrupts()
#define __NOP() halt_cpu()
''' + hook + r'''
int main(void){
    if(setjmp(halted)==0)vApplicationStackOverflowHook(NULL,NULL);
    assert(sequence==2);puts("stack hook closes output before halt, without scheduler/log IO");
}
''', 'fault_hook')


if __name__ == '__main__':
    main()
