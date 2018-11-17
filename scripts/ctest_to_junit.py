from lxml import etree
import StringIO
import sys

TAGfile = open(sys.argv[1]+"/Testing/TAG", 'r')
dirname = TAGfile.readline().strip()

xmlfile = open(sys.argv[1]+"/Testing/"+dirname+"/Test.xml", 'r')
xslfile = open(sys.path[0] + "/CTest2JUnit.xsl", 'r')

xmlcontent = xmlfile.read()
xslcontent = xslfile.read()

xmldoc = etree.parse(StringIO.StringIO(xmlcontent))
xslt_root = etree.XML(xslcontent)
transform = etree.XSLT(xslt_root)

result_tree = transform(xmldoc)
result_tree.write(sys.argv[1]+"/Testing/"+dirname+"/jUnit.xml")