#error out if no html directory

function stringcat(){
echo "catting";
CHARSTRING=$CHARSTRING$(cat -);
}


#todo: iterate through eac
HTML_FILES=./html/*.html
HEADER_PATH="./main/"
#delete all generated headers that exist
rm ${HEADER_PATH}/*.html.hh

for file in ${HTML_FILES}; do
  HTML_FILENAME="${file}"
  HEADER_NAME_BASE=`echo ${file} | cut -d '/' -f3 | cut -d '.' -f1`
  HEADER_NAME_BASE_UPPER=`echo $HEADER_NAME_BASE | tr /a-z/ /A-Z/`
  HEADER_FILENAME=${HEADER_PATH}${HEADER_NAME_BASE}.html.hh

  echo "processing file: ${HTML_FILENAME}"

  echo "#ifndef ${HEADER_NAME_BASE_UPPER}_HTML_HH" >  ${HEADER_FILENAME}
  echo "#define ${HEADER_NAME_BASE_UPPER}_HTML_HH" >> ${HEADER_FILENAME}
  echo "/************************************************" >> ${HEADER_FILENAME}
  echo " @file ${HEADER_FILENAME}" >> ${HEADER_FILENAME}
  echo " * GENERATED FILE -- DO NOT HAND-MODIFY!!!!!!!!!!" >> ${HEADER_FILENAME}
  echo " ***********************************************/" >> ${HEADER_FILENAME}
  echo " /*! @var ${HEADER_NAME_BASE}_text_0" >> ${HEADER_FILENAME}
  echo "  *  @brief generated variable  ${HEADER_NAME_BASE}_text_0" >> ${HEADER_FILENAME}
  echo "  */ " >> ${HEADER_FILENAME}
  echo "" >> ${HEADER_FILENAME}
  echo "#include <string>" >> ${HEADER_FILENAME}
  echo "const std::string ${HEADER_NAME_BASE}_raw = R\"(" >> ${HEADER_FILENAME}

  # Write the lines of the HTML file to the header file:
  #   * Skip lines which start with "//" (javascript comments must be at beginning of line)
  #   * Skip lines which start with "<!--" (HTML Comments must be at beginning of line)
  #   * Strip off leading tabs and spaces (to save memory space)
  grep -v "^//" ${HTML_FILENAME} | grep -v "^<--" | sed "s/^[ \t]*//" >> ${HEADER_FILENAME}
  echo ")\";" >> ${HEADER_FILENAME}

  echo "finalizing"
  
  echo ""  >> ${HEADER_FILENAME}
  echo "#endif"  >> ${HEADER_FILENAME}

done
