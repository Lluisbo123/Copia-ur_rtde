import os
os.add_dll_directory("C:/CodingXP/msvc17_64/usr/lib")


import rtde_control
import rtde_receive
import script_client
import time


ip = "192.168.190.134"
script_client = script_client.ScriptClient(ip, 5, 2)
rtde_c = rtde_control.RTDEControlInterface(ip)
script_client.connect()
print("Is connected: ", script_client.isConnected())
print("rtde_c.isConnected: ", rtde_c.isConnected())

command = (
    "def script_test():\n"
        "\tdef test():\n"
            "textmsg(\"test1\")\n"
            "textmsg(\"test2\")\n"
        "\tend\n"
        "\twrite_output_integer_register(0, 1)\n"
        "\ttest()\n"
        "\ttest()\n"
        "\twrite_output_integer_register(0, 2)\n"
    "end\n"
    "run program\n")

thread_command =(
    "def script_test():\n"
        "\tglobal state = 0\n"
        "\tthread test():\n"
            "\ttextmsg(\"test thread started\")\n"
            "\ttextmsg(\"test thread finished\")\n"
        "\tend\n"
        "\twrite_output_integer_register(0, 1)\n"
        "\ttest_thrd = run test()\n"
        "\tjoin test_thrd\n"
        "\twrite_output_integer_register(0, 2)\n"
    "end\n"
    "run program\n")

print("starting script...")
#Result = rtde_c.sendCustomScriptFunction("function_test", "textmsg(\test\)")
Result = rtde_c.sendCustomScript(command)
#Result = rtde_c.sendCustomScriptFile("testscript.script")

print("Result: ", Result)

script_client.disconnect()