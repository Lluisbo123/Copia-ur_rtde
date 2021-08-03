#!/usr/bin/env python
import dashboard_client 
import time 

time.sleep(1)
db_client = dashboard_client.DashboardClient("localhost", 29999, True)
db_client.connect()
db_client.brakeRelease()
