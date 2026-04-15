# Appendix: sentai.fs

The `sentai.fs` namespace provides script-level access to the LittleFS user partition. It is used to move models, scripts, logs, images, and outputs between the runtime and persistent flash, and it is a central part of the interactive workflow because both model deployment and experiment logging depend on it.

## Functions

- `sentai.fs.read(path)`: Reads an entire file as bytes.
- `sentai.fs.read_str(path)`: Reads an entire file as text.
- `sentai.fs.read_base64(path)`: Reads and prints a file as base64 text.
- `sentai.fs.write(path, data)`: Writes bytes or text to flash.
- `sentai.fs.size(path)`: Returns the file size.
- `sentai.fs.exists(path)`: Checks whether a file or directory exists.
- `sentai.fs.remove(path)`: Deletes a file or empty directory.
- `sentai.fs.mkdir(path)`: Creates directories recursively.
- `sentai.fs.ls(path)`: Lists directory contents.
- `sentai.fs.format()`: Reformats the user partition and erases user data.

## Example

```python
import sentai

sentai.fs.mkdir('/logs')
sentai.fs.write('/logs/run.txt', 'experiment started\n')
print(sentai.fs.read_str('/logs/run.txt'))
print(sentai.fs.ls('/logs'))
```