import os
from pathlib import Path

def main():
    # Папка, где лежит этот скрипт
    script_path = Path(__file__).resolve()
    base_dir = script_path.parent

    # Имя выходного файла
    output_file = base_dir / "selected_files_dump.txt"

    # Какие расширения собираем
    allowed_ext = {".py", ".wbt"}

    with output_file.open("w", encoding="utf-8") as out:
        out.write(f"Базовая папка: {base_dir}\n")
        out.write("=" * 80 + "\n\n")

        # Рекурсивный обход всех файлов и папок
        for root, dirs, files in os.walk(base_dir):
            root_path = Path(root)

            for name in files:
                file_path = root_path / name

                # Пропускаем сам скрипт и файл вывода
                if file_path == script_path or file_path == output_file:
                    continue

                # Фильтр по расширению
                if file_path.suffix.lower() not in allowed_ext:
                    continue

                out.write(f"ФАЙЛ: {file_path.relative_to(base_dir)}\n")
                out.write("-" * 80 + "\n")

                # Пытаемся прочитать как текст
                try:
                    with file_path.open("r", encoding="utf-8", errors="replace") as f:
                        content = f.read()
                    out.write(content)
                except Exception as e:
                    out.write(f"[ОШИБКА ЧТЕНИЯ ФАЙЛА: {e}]\n")

                out.write("\n" + "=" * 80 + "\n\n")

    print(f"Готово! Результат записан в: {output_file}")

if __name__ == "__main__":
    main()
