#!/usr/bin/env python
import dashboard_client 
import time 

time.sleep(1)
db_client = dashboard_client.DashboardClient("192.168.56.101", 29999, True)
db_client.connect()
db_client.brakeRelease()
