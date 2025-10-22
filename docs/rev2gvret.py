


if __name__  == "__main__":

  import argparse
  import json
  
  parser = argparse.ArgumentParser(description='REV to GVRET conversion routine.')
  parser.add_argument('src_filename', type=str, help='Input filename')
  parser.add_argument('dest_filename', type=str, help='Input filename')
  parser.add_argument('-n', '--skip_initial_lines', type=int, help='Number of lines to skip at start of file.', default=0)
  parser.add_argument('-t', '--skip_final_lines', type=int, help='Number of lines to skip at end of file.', default=0)
  args = parser.parse_args()
  
  header = "Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\n"

  numLines = sum(1 for _ in open(args.src_filename))

  fIn = open(args.src_filename);
  fOut = open(args.dest_filename, 'w');
  fOut.write(header)
  print(args.skip_initial_lines)
  for i in range(numLines):
    ln = fIn.readline()
    if (i > args.skip_initial_lines) and (i < numLines - args.skip_final_lines):
      fOut.write(ln.replace(' ', ','))

  fIn.close()
  fOut.close()
    


  
