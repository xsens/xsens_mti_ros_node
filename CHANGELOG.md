# Changelog for package xsens_mti_ros_node

## 4.0.2 (2018-07-31)
* Updated gps package required in prerequisites section of README

## 4.0.1 (2018-07-20)
* Fixed typo in xsens.yaml file w.r.t MTi-3 filter profiles
* Changed example message in ReadMe

## 4.0.0 (2018-07-04)
* Merge pull request #33: Change custom_msgs to xsens_msgs
* Merge pull request #32: Configure MTi device on startup
* Added support to MTi-7 device

## 3.0.2 (2018-04-24)
* Added warning (rospy.logwarn) for timeouts

## 3.0.1 (2017-08-09)
* Support added to the node for operation with Kinetic environment

## 3.0.0 (2016-05-27)
* Added functionality to detect devices based on Product masks (incl. 1-series and FMT1000 devices)
* Increased the 'additionalTimeOutOffset' from 6ms to 10ms as default

## 2.0.2 (2015-11-23)
* Fix Readme

## 2.0.1 (2015-03-19)
* Fix documentation, switch to Markdown

## 2.0.0 (2015-03-19)
* Updated GNSS data identifiers
* Fixed a timeout issue
* Added new message files
* Add baudrate checks
* Add filter options
* Removed support for legacy devices

## 1.0.0 (2014-09-02)
* Improved on MK4 functionality to publish /xsens/sensorSample messages

Built on ethzasl_xsens_driver developed by previous contributors - Enrique Fernandez, Francis Colas, Paul Mathieu, Sam Pfeiffer, 
Benjamin Hitov, Francis Colas, Nikolaus Demmel, Stéphane Magnenat, fcolas.
