# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "GitPython",
# ]
# ///
import git
from datetime import datetime
from pathlib import Path

filename = "../Core/Inc/git-hash.h"
contents = """/*
 *  git-hash.h
 *
 *  Auto-generated during the STM32 pre-build steps
 *  Last generated: {time}
 */

#ifndef INC_GIT_HASH_H_
#define INC_GIT_HASH_H_

#define GIT_HASH        "{hash}"
#define GIT_BRANCH      "{branch}"
#define BUILD_TIME      "{time}"
#define BUILD_TYPE      "{build_type}"

#endif /* INC_GIT_HASH_H_ */"""

repo = git.Repo(search_parent_directories=True)
branch_name = repo.active_branch
short_hash = repo.git.rev_parse('--short', 'HEAD')

build = Path.cwd().name

time_pretty = datetime.now().astimezone().strftime("%b %d %Y %I:%M:%S %p %Z")

with open(filename, "w") as file:
    file.write(contents.format(time = time_pretty, hash = short_hash, branch = branch_name, build_type = build))