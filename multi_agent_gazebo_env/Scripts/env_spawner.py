import os
print os.getcwd()
os.chdir(os.path.join('/', *(__file__.split('/')[:-1])))
print os.getcwd()
print __file__.split('/')