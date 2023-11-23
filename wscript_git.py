# encoding: utf-8
from waflib.Configure import conf
from waflib import Context, Logs, Task, Utils

def _git_head_hash(ctx, path, short=False):
    cmd = [ctx.env.get_flat('GIT'), 'rev-parse']
    if short:
        cmd.append('--short=8')
    cmd.append('HEAD')
    out = ctx.cmd_and_log(cmd, quiet=Context.BOTH, cwd=path)
    return out.strip()

@conf
def git_head_hash(self, short=False):
    return _git_head_hash(self, self.srcnode.abspath()+'/..', short=short)
