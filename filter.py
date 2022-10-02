import re
import sys, getopt


def main(argv):
   inputfile = ''
   outputfile = ''
   try:
      opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
   except getopt.GetoptError:
      print 'filter.py -i <inputfile> -o <outputfile>'
      sys.exit(2)
   for opt, arg in opts:
      if opt == '-h':
         print 'filter.py -i <inputfile> -o <outputfile>'
         sys.exit()
      elif opt in ("-i", "--ifile"):
         inputfile = arg
      elif opt in ("-o", "--ofile"):
         outputfile = arg
   print 'Input file is "', inputfile
   print 'Output file is "', outputfile
   return inputfile,outputfile

if __name__== "__main__":
   regex = r"^\s+condBranches:\s+(?P<condBranch>\d+)"
   regex1 = r"^\s+westmere-(?P<westmere>\d+)"
   regex2 = r"^\s+cycles:\s+(?P<cycles>\d+)"
   regex3 = r"^\s+instrs:\s+(?P<instrs>\d+)"

   list = ['condBranch','cycles','instrs','mispredBranches','cCycles']


   inputfile,outputfile = main(sys.argv[1:])
   outputfile = open(outputfile, "w")
   outputfile.write(inputfile+"\n")
   print(inputfile)
   with open(inputfile) as inputfile:
      for line in inputfile:
         match1 = re.search(regex1,line)
         if match1:
            westmere = match1.group('westmere')
            outputfile.write("westmere "+westmere+"\n")
         else:
            for i in list:
               regex = r"^\s+"+i+":\s+(?P<"+i+">\d+)"
               match = re.search(regex,line)
               if match:
                  tmp = match.group(i)
                  outputfile.write(i+" "+tmp+"\n")
   outputfile.close()
