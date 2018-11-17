<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:output method="xml" indent="yes" />
	<xsl:template match="/">
		<testsuites>
			<xsl:variable name="buildName" select="//Site/@BuildName"/>
			<xsl:variable name="numberOfTests" select="count(//Site/Testing/Test)"/>
			<xsl:variable name="numberOfFailures" select="count(//Site/Testing/Test[@Status!='passed'])" />
			<testsuite name="CTest"
				tests="{$numberOfTests}" time="0"
				failures="{$numberOfFailures}"  errors="0"
				skipped="0">
			<xsl:for-each select="//Site/Testing/Test">
					<xsl:variable name="testName" select="translate(Name, '-', '_')"/>
					<xsl:variable name="duration" select="Results/NamedMeasurement[@name='Execution Time']/Value"/>
					<xsl:variable name="status" select="@Status"/>
					<xsl:variable name="output" select="Results/Measurement/Value"/>
					<xsl:variable name="className" select="translate(Path, '/.', '.')"/>
					<testcase classname="projectroot{$className}"
						name="{$testName}"
						time="{$duration}">
						<xsl:if test="@Status!='passed'">
							<failure>
								<xsl:value-of select="$output" />
							</failure>
						</xsl:if>
						<system-out>
							<xsl:value-of select="$output" />
						</system-out>
					</testcase>
				</xsl:for-each>
			</testsuite>
		</testsuites>
	</xsl:template>
</xsl:stylesheet>
