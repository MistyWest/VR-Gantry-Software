# SOFTWARE - HTC Vive Gantry #

![](https://cdn-images-1.medium.com/max/800/1*L0-0M3ktUiBQcZHqOwzkyA.gif)

Send gantry positioning through Serial according to position and orientation of VR headset.

### Dependencies ###
[Software Repository](https://github.com/MistyWestAdmin/VR-Gantry-Software)<br>
[Firmware Repository](https://github.com/MistyWestAdmin/VR-Gantry-Firmware)

### Configuration ###

* Dependencies: OpenVR

* Project Configuration Properties for Visual Studios
    * General
        * Configuration Type: Application (.exe)
        * Character Set: Use Multi-Byte Character Set
    * VC++ Directories
        * Include Directories: ~\openvr\headers
        * Library Directories: ~\openvr\lib\win64
    * Linker > Input
        * Additional Dependencies: ~\openvr\lib\win64\openvr_api.lib

### Contact ###

* Repo owner: Justin
* Contributors: Walker, Div
* [Mistywest](https://mistywest.com/)
<br><br>
![](https://mistywest.com/wp-content/uploads/2015/11/logo_sticky.png)
<br>
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.