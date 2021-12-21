# Python 3 example

This is a Python 3 example. Note that this example requires NumPy, SciPy, and
[Open3D](https://github.com/isl-org/Open3D) for IO and visualization. You can
use the following command to install them:

```bash
python3 -m pip install -r requirements.txt
```

## :running: Execution

```bash
python3 main.py
```

The main function will read two point clouds from [data](../data), perform the
scan matching using KCP, and visualize the registration result with Open3D.

If you receive the message:

```
Oops, We cannot import pykcp!
Make sure that you have properly compiled the python binding of KCP.
```

it means that you do not properly build the Python binding of KCP (pykcp) under
`../../build` relative to this directory (there should be a file something likes
`../../build/python/pykcp.cpython-XXX-XXX-XXX-XXX.so`). Please check the
installation guideline and build KCP **with python binding**.
