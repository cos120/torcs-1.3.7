from sys     import stderr
from logging import getLogger, StreamHandler, Formatter, DEBUG

l  = getLogger()
sh = StreamHandler(stderr)
sh.setLevel(DEBUG)
f  = Formatter('%(asctime)s\t%(message)s')
sh.setFormatter(f)
l.addHandler(sh)
l.setLevel(DEBUG)

L0 = ['abc', 'def', 'ghi']
L1 = ['jkl', 'mno', 'pqr']
log_info_field = ['episode','step','reward','adv_reward','env_reward','current_adv_speed','total_reward']
write = list(range(7))
msg = zip(log_info_field,write)
a = ['{}'] * 7
b = '\t'.join(a)
c = b.format(*map(lambda x:'{} {}'.format(*x),msg))
print(c)
# identical to 
l.info('%s\t%s', L0, L1)
# identical to 
