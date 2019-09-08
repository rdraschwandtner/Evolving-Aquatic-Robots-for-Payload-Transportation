
import multiprocessing as mpc
import os

# Workaround for global variable
import GlobalVarWorkaroundModule

#########################################################################
# check this
# https://docs.python.org/2/library/multiprocessing.html#multiprocessing-programming
# it contains a windows guideline!
#########################################################################

# Bear in mind that if code run in a child process tries to access a global variable,
# then the value it sees (if any) may not be the same as the value in the parent process
# at the time that Process.start was called.
globalvar = 'global' # Code guideline!, might be different in processes


class moduletest_t:
    def __init__(self):
        self.classvar = 'classvar'

moduletest_instance = moduletest_t()

def some_fun(param):
    # do stuff

    pid = os.getpid()
    return param + '.. pid:' + `pid` + ' globalvar=' + globalvar + ' classvar=' + moduletest_instance.classvar + ' modulevar=' + GlobalVarWorkaroundModule.toShare
    #return param + '.. pid:' + `pid` + ' globalvar=' + globalvar + ' classvar=' + moduletest_instance.classvar + ' modulevar=' + toShare

################################
# Workaround for global variable
# http://stackoverflow.com/questions/1675766/how-to-combine-pool-map-with-array-shared-memory-in-python-multiprocessing
################################
def initProcess(share):
  GlobalVarWorkaroundModule.toShare = share


# .. one should protect the entry point of the program by using if __name__ == '__main__': as follows:
if __name__ == '__main__': # Code guideline!
    # test MPC pool
    print('Hello World')
    #print some_fun('myparam ')

    globalvar = 'overwritten'
    moduletest_instance.classvar = 'overwritten classvar'
    GlobalVarWorkaroundModule.toShare = 'overwritten initstate'


    params = []
    params.append('first param')
    params.append('second param')
    params.append('third param')
    params.append('fouth param')

    results = []

    # serial execution
    #for i in range(len(params)):
    #    results.append(some_fun(params[i]))

    #cores = mpc.cpu_count()
    #pool = mpc.Pool(processes=2)
    # global var workaround
    pool = mpc.Pool(initializer=initProcess, initargs=(GlobalVarWorkaroundModule.toShare,), processes=2)
    results = pool.map(some_fun,params)

    print results

    print 'end'


