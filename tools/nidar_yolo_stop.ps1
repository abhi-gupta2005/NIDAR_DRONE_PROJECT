# NIDAR: stop nidar_v11_windows.py (used by tools/nidar_run.sh before a run and when it stops).
Get-CimInstance Win32_Process |
    Where-Object { $_.CommandLine -like '*nidar_v11_windows.py*' } |
    ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
