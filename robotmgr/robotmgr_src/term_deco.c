#include <stdio.h>

#if defined(_WIN32)
#include <windows.h>
#include <tlhelp32.h>
#else
#include <unistd.h>
#endif

#if defined(_MSC_VER)
#pragma warning(push, 4)
#endif

#if defined(_DEBUG)
#define STATIC
#else
#define STATIC  static
#endif

#ifndef UNREFERENCED_PARAMETER
#define UNREFERENCED_PARAMETER(__var)   ((__var) = (__var))
#endif


#if defined(_WIN32)


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_TERM_GetProcessEntry()
//

static BOOL _loc_TERM_GetProcessEntry(PROCESSENTRY32* pEntry)
{
    HANDLE  hSnapShot;
    DWORD pid;

    // get a handle to a Toolhelp snapshot of the systems processes.
    hSnapShot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);

    if (hSnapShot == INVALID_HANDLE_VALUE)
    {
        return FALSE;
    }

    // get the first process' information.
    ZeroMemory(pEntry, sizeof(PROCESSENTRY32));

    pEntry->dwSize = sizeof(PROCESSENTRY32) ;
    if (!Process32First(hSnapShot, pEntry))
        return FALSE;

    // while there are processes, keep looping.
    pid = GetCurrentProcessId();

    while (pid != pEntry->th32ProcessID)
    {
        pEntry->dwSize = sizeof(PROCESSENTRY32) ;
        if (!Process32Next(hSnapShot, pEntry))
            break;
    }

   return pid == pEntry->th32ProcessID;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_TERM_GetParentProcessId()
//

static DWORD _loc_TERM_GetParentProcessId(void)
{
    PROCESSENTRY32 entry;

    if (!_loc_TERM_GetProcessEntry(&entry))
        return 0;

    return entry.th32ParentProcessID;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: _loc_TERM_EnumConsoleWindowsProc()
//

static HWND s_hWndCurrentConsole = NULL;

static BOOL CALLBACK _loc_TERM_EnumConsoleWindowsProc(HWND hWnd, LPARAM lParam)
{
    DWORD dwThisProcessId = (DWORD) lParam;

    char szClassName[100];
    DWORD dwWndProcess;

    if (GetClassName(hWnd, szClassName, sizeof(szClassName)) == 0)
        return TRUE;

    if (strcmp(szClassName, "ConsoleWindowClass") != 0)
        return TRUE;

    GetWindowThreadProcessId(hWnd, &dwWndProcess);

    if (dwWndProcess != dwThisProcessId)
        return TRUE;

    s_hWndCurrentConsole = hWnd;

    return FALSE;
}


/////////////////////////////////////////////////////////////////////////////
//
//  Function: TERM_SetCurrentConsoleIcon()
//

int TERM_SetCurrentConsoleIcon(HICON hIconSmall,
                               HICON hIconBig,
                               HICON* phSmallOld,
                               HICON* phBigOld)
{
    DWORD dwThisProcessId;
    int nResult;

    if (s_hWndCurrentConsole == NULL)
    {
        dwThisProcessId = GetCurrentProcessId();
        EnumWindows(_loc_TERM_EnumConsoleWindowsProc, (LPARAM) dwThisProcessId);

        if (s_hWndCurrentConsole == NULL)
        {
            dwThisProcessId = _loc_TERM_GetParentProcessId();
            EnumWindows(_loc_TERM_EnumConsoleWindowsProc, (LPARAM) dwThisProcessId);
        }
    }

    if (s_hWndCurrentConsole == NULL)
    {
        hIconSmall = NULL;
        hIconBig   = NULL;

        nResult = -1;
    }
    else
    {
        if (hIconBig != NULL)
        {
            hIconBig   = (HICON) SendMessage(s_hWndCurrentConsole,
                                             WM_SETICON,
                                             (WPARAM) ICON_BIG,
                                             (LPARAM) (HICON) hIconBig);
        }

        if (hIconSmall != NULL)
        {
            hIconSmall = (HICON) SendMessage(s_hWndCurrentConsole,
                                             WM_SETICON,
                                             (WPARAM) ICON_SMALL,
                                             (LPARAM) (HICON) hIconSmall);
        }

//        UpdateWindow(s_hWndCurrentConsole);
        nResult = 0;
    }

    if (phSmallOld != NULL)
        *phSmallOld = hIconSmall;

    if (phBigOld != NULL)
        *phBigOld = hIconBig;

    return nResult;
}

#else

///
#endif

