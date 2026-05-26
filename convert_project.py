import os
import traceback
import fnmatch

# ================= НАСТРОЙКИ =================
OUTPUT_FOLDER = 'txt'
TREE_FILENAME = 'project_tree.txt'
COMBINED_FILENAME_BASE = 'all_code_part' # Имя будет: all_code_part_1.txt и т.д.
HEADERS_FILENAME = 'all_headers.txt'     # Имя файла только для заголовков

# Максимальный размер одного выходного файла в байтах (950 КБ для запаса)
MAX_OUTPUT_FILE_SIZE = 950 * 1024 

# Максимальный размер ОДИНОЧНОГО файла исходника, который мы берем
MAX_SOURCE_FILE_SIZE = 500 * 1024 

MINI_IGNORE = {
    '*.exe', '*.dll', '*.so', '*.dylib', '*.bin',
    '*.jpg', '*.jpeg', '*.png', '*.gif', '*.svg', '*.ico',
    '*.pdf', '*.doc', '*.docx', '*.xls', '*.xlsx',
    '*.zip', '*.rar', '*.7z', '*.tar', '*.gz',
    '*.mp3', '*.mp4', '*.wav', '*.avi',
    '*.pyc', '__pycache__', '.git', '.idea', '.vscode', '.vs',
    'node_modules', 'build','build_all', 'build-gcc','3rdparty', 'dist', 'out', 'debug', 'release',
    '*.pdb', '*.obj', '*.o', '*.lib', '*.a', '*.stp', '*.stl','*.step', '*.ttf',
    'package-lock.json', 'yarn.lock', 
    'HexaStudio/meshes','HexaStudio/resources', 'Shared/nlohmann'}
# =============================================

def load_gitignore_patterns(base_path):
    patterns = set()
    gitignore_path = os.path.join(base_path, '.gitignore')
    if os.path.exists(gitignore_path):
        try:
            with open(gitignore_path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        patterns.add(line.rstrip('/'))
        except: pass
    return patterns

def should_ignore(rel_path, patterns):
    filename = os.path.basename(rel_path)
    for p in MINI_IGNORE:
        if fnmatch.fnmatch(filename, p) or fnmatch.fnmatch(rel_path, p):
            return True
        if p.startswith('*.'):
            if filename.endswith(p[1:]): return True
    for pattern in patterns:
        if fnmatch.fnmatch(rel_path, pattern) or fnmatch.fnmatch(filename, pattern) or \
           any(fnmatch.fnmatch(part, pattern) for part in rel_path.split(os.sep)):
            return True
    return False

def generate_tree(startpath, git_patterns):
    tree_str = []
    for root, dirs, files in os.walk(startpath):
        rel_root = os.path.relpath(root, startpath)
        if rel_root == ".": rel_root = ""
        dirs[:] = [d for d in dirs if not should_ignore(os.path.join(rel_root, d), git_patterns) and d != OUTPUT_FOLDER]
        level = rel_root.count(os.sep) if rel_root else 0
        indent = '    ' * level
        tree_str.append(f"{indent}|-- {os.path.basename(root) or os.path.basename(startpath)}/")
        for f in files:
            if not should_ignore(os.path.join(rel_root, f), git_patterns) and f != os.path.basename(__file__):
                tree_str.append(f"{'    ' * (level + 1)}|-- {f}")
    return "\n".join(tree_str)

def main():
    try:
        base_path = os.getcwd()
        output_path = os.path.join(base_path, OUTPUT_FOLDER)
        if not os.path.exists(output_path): os.makedirs(output_path)

        git_patterns = load_gitignore_patterns(base_path)
        tree_content = generate_tree(base_path, git_patterns)
        
        # 1. Сохраняем файл с деревом
        with open(os.path.join(output_path, TREE_FILENAME), 'w', encoding='utf-8') as f:
            f.write(tree_content)

        # 2. Открываем файл для заголовков (.h)
        headers_path = os.path.join(output_path, HEADERS_FILENAME)
        f_headers = open(headers_path, 'w', encoding='utf-8')
        f_headers.write("=== PROJECT HEADERS (Tree + .h files) ===\n")
        f_headers.write("Structure:\n" + tree_content + "\n\n")

        print("Сборка файлов с учетом лимита размера...")
        
        part_num = 1
        current_combined_size = 0
        
        # Функция для открытия нового файла-части (для основного кода)
        def open_new_part(num):
            fname = f"{COMBINED_FILENAME_BASE}_{num}.txt"
            f = open(os.path.join(output_path, fname), 'w', encoding='utf-8')
            f.write(f"=== PROJECT CONTEXT PART {num} ===\n")
            f.write("Structure (summary):\n" + tree_content[:1000] + "...\n\n") 
            return f

        combined_f = open_new_part(part_num)

        count = 0
        headers_count = 0

        for root, dirs, files in os.walk(base_path):
            rel_root = os.path.relpath(root, base_path)
            if rel_root == ".": rel_root = ""
            dirs[:] = [d for d in dirs if not should_ignore(os.path.join(rel_root, d), git_patterns) and d != OUTPUT_FOLDER]

            for filename in files:
                if filename in {os.path.basename(__file__), TREE_FILENAME, HEADERS_FILENAME}: continue
                
                rel_file_path = os.path.join(rel_root, filename)
                if should_ignore(rel_file_path, git_patterns): continue

                src_full_path = os.path.join(root, filename)
                
                # Проверка размера одиночного файла
                file_size = os.path.getsize(src_full_path)
                if file_size > MAX_SOURCE_FILE_SIZE:
                    print(f"Пропуск: {rel_file_path} (слишком большой: {file_size//1024} КБ)")
                    continue

                try:
                    with open(src_full_path, 'r', encoding='utf-8', errors='replace') as f_src:
                        content = f_src.read()
                    
                    file_header = f"\n\n--- START OF FILE: {rel_file_path} ---\n"
                    file_footer = f"\n--- END OF FILE: {rel_file_path} ---\n"
                    total_entry = file_header + content + file_footer
                    
                    # === ЛОГИКА ДЛЯ ОСНОВНОГО СБОРНИКА ===
                    if current_combined_size + len(total_entry.encode('utf-8')) > MAX_OUTPUT_FILE_SIZE:
                        combined_f.close()
                        part_num += 1
                        combined_f = open_new_part(part_num)
                        current_combined_size = 0
                        print(f"\nСоздана часть {part_num}...")

                    combined_f.write(total_entry)
                    current_combined_size += len(total_entry.encode('utf-8'))
                    
                    # === ЛОГИКА ДЛЯ ФАЙЛА ЗАГОЛОВКОВ (.h) ===
                    # Добавил также .hpp на всякий случай, если не нужно - удалите "or filename.lower().endswith('.hpp')"
                    if filename.lower().endswith('.h') or filename.lower().endswith('.hpp'):
                        f_headers.write(total_entry)
                        headers_count += 1

                    # === СОХРАНЕНИЕ ОТДЕЛЬНОГО ФАЙЛА ===
                    safe_name = rel_file_path.replace(os.sep, '_').replace('.', '_') + ".txt"
                    with open(os.path.join(output_path, safe_name), 'w', encoding='utf-8') as f_dest:
                        f_dest.write(content)

                    count += 1
                    if count % 10 == 0: print(f"Обработано файлов: {count}", end='\r')
                except: pass

        combined_f.close()
        f_headers.close() # Закрываем файл заголовков

        print(f"\n\nГОТОВО!")
        print(f"Всего файлов: {count}")
        print(f"Из них заголовков (.h): {headers_count}")
        print(f"Создано частей с кодом: {part_num}")
        print(f"Файл заголовков: {HEADERS_FILENAME}")
        print(f"Ищите файлы в папке {OUTPUT_FOLDER}")

    except Exception:
        traceback.print_exc()

if __name__ == "__main__":
    main()
    input("\nНажмите Enter...")