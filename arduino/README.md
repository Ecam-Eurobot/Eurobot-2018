# Arduino code

## ecamlib

`ecamlib` is an Arduino library containing utility functions
that can be used for different codes. Currently it contains
a couple of functions to converts raw bytes to `int`, `float`
and `long` types.

#### Install

To be able to use the library, you need to install it first.
The Arduino IDE expects libraries to be located in a
subfolder in the default sketch directory. For example
`~/Documents/Arduino/libraries/` or `My Documents\Arduino\libraries\`.

There are multiple ways to install the library:

1. **ZIP**
   The easiest solution is to zip the `ecamlib` folder
   located in this repository and install it through the
   Arduino IDE using the menu entry: `Sketch > Include Library > Add .ZIP Library ...`
   This method is simple, but there are two things to keep in mind:
   - If the code of the library changes, you need to do this
     procedure again to have the latest changes.
   - If you make the .zip in the repository, make sure to not
     commit it when you make changes to other code.
2. **Copy**
   The second solution is to copy the ecamlib folder to the
   Arduino library folder, a subdirectory called `libraries`
   in your default sketch directory. With this method, you
   need to redo this procedure whenever a change is made to
   the library.
3. **Symlink**
   The symlink method is a little bit trickier, especially on Windows,
   but it is the more robust solution. The idea, is to create
   a symlink in the Arduino library that points to the `ecamlib`
   folder in this repository.

   **Windows:**
   Open the command prompt and type:
   ```
   mklink /D "C:\path\to\Arduino\libraries" "C:\path\to\ecamlib"
   ```
   **Linux/MacOS:**
   Open the terminal and type:
   ```
   sudo ln -s ~/path/to/ecamlib ~/path/to/Arduino/libraries/ecamlib
   ```

#### Usage
When the library is installed in the Arduino IDE, you should be
able to use it by selecting `Sketch > Include Library > ecamlib`
You will probably need to scroll to the end of the list.

It will add an include in the source file for you:

```cpp
#include <ecamlib.h>
```