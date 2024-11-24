#!/usr/bin/python

import re, sys
from subprocess import Popen, PIPE, STDOUT

try:
  from subprocess import DEVNULL  # py3k
except ImportError:
  import os
  DEVNULL = open(os.devnull, "wb")


def runCmd(arguments):
  arguments = arguments.split(" ")
  try:
    process = Popen(arguments,\
    stdout=PIPE, stderr=DEVNULL, universal_newlines=True)

    output, error = process.communicate()
    # process.terminate()
    return output
  except Exception as e:
    print('CMD fail {}'.format(e))


def getBugId(log):
  ids = []
  for line in log.split("\n"):
    #m = re.match(r"[Bb][Uu][Gg]\:(.*)", line.strip())
    m = re.match(r"[B][u][g]\:(.*)", line.strip())
    if m:
      ids.append(m.group(1).strip())
  return ids


def generateBugIdList(ids, m, bugList):
  if (len(ids) > 0):
    index = 0
    for id in ids:
      if index == 0:
        # Concat the first id with the message in the Merge List
        bug = 'Bug: {}'.format(id) + " (ACK)"
      else:
        bug = 'Bug: {}'.format(id) + " (ACK)"

      index = index + 1
      bugList.append(bug)
  else:
    # No bug id in the commit
    # print ">>>>> No Bug ID in " + m.group(0)
    pass


def getMergeInfo(info):
  start = ""
  end = ""

  for line in (info):
    line = line.strip().split(" ")
    if (line[0] == "Merge:"):
      start = line[1]
      end = line[2]

  return (start, end)


def genCommitInfo(bugList):
  commitText = "CommitInfo.txt"
  with open (commitText, 'a') as f:

    for bug in (bugList):
      f.write(bug + "\n")


# Main Function
mergeInfo = runCmd("git log -1").split("\n")
(start, end) = getMergeInfo(mergeInfo)

lines = runCmd("git log --oneline " + start + ".." + end).split("\n")
BugList = []
for index, line in enumerate(lines):
  #print line
  # Expect the output sha length is longer than 7 digits
  m = re.match(r"([0-9a-fA-F]{7,}) (.*)", line.strip())
  if m:
    gitLog = runCmd("git log -1 " + m.group(1))
    if (gitLog is not None):
      ids = getBugId(gitLog)
      generateBugIdList(ids, m, BugList)

BugList = list(dict.fromkeys(BugList))
BugList.sort()
genCommitInfo(BugList)
