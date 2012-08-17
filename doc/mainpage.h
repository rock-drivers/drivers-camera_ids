/** \mainpage IDS-Camera driver
 *
 * - \subpage intro
 * - \subpage settings
 * - \subpage test
 *
 */

/** \page intro Introduction
 *
 * Drivers for cameras from <a href="http://www.ids-imaging.de">IDS-Imaging</a> based
 * on their sdk and.
 * A manual can be found <a href="http://www.ids-imaging.de/frontend/files/
 * uEyeManuals/Manual_eng/uEye_Manual/index.html">here</a>.
 */

/** \page settings Settings
 *
 * - \b Pixelclock defines how often the pixels are read.
 * - \b Exposure is the time the chip is exposed to light. Value depends on
 *   pixelclock and is set in ms.
 * - \b Framerate's max possible value depends on the pixelclock and exposure.
 * - \b AOI is to get a samller size of the picture (allows increased framerate)
 * - \b Binning reads only every nth pixel (allows increased framerate)
 *
 * Some settings for some cameras.
 * \section UI122xLE-M
 * - Pixelclock 5 - 40 MHz
 *  - 5 MHz:
 *   - Framerate 0.18 - 10.89 fps
 *   - Exposure 0.64 - 132.97 ms
 *  - 20 MHz:
 *   - Framerate 0.72 - 43.57 fps
 *   - Exposure 0.16 - 33.24 ms
 *  - 40 MHz:
 *    - Framerate 1.43 - 87.15 fps
 *    - Exposure 0.08 - 16.62 ms
 * - Binning None, 2x, 4x
 *   - 2x, 40 MHz Pixelclock: max. 159.27 fps
 *   - 4x, 40 MHz Pixelclock: max. 271.70 fps
 *   - 4x, 60 MHz Pixelclock: max. 407.55fps
 *  */

/** \page test Test Program
 * 
 * To test the driver use test_bin in the build/test folder.
 * \code
 * ./test_bin --help
 * \endcode
 *
 * Also use it to see how the driver might be used.
 */
