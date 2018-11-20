<?xml version="1.0" encoding="UTF-8"?>
<!-- Copyright 2018 The Cartographer Authors

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

	  http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
-->
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
