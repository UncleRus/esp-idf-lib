#!/usr/bin/env ruby

require "fileutils"

# Rewrites Kconfig.projbuild for ESP8266. Replaces `rsource "path"` with the
# content of the `path`.

def rewrite(dir, kconfig_file)
  new_content = ""
  Dir.chdir(dir) do
    File.readlines(kconfig_file).each do |line|

      # XXX hackish parser
      if line =~ /^\s+rsource\s+/
        _ignore, file = line.gsub(/^\s+/, "").gsub(/[ ]+/, " ").split(" ")
        file.gsub!(/"/, "")
        new_content += File.read(file)
      else
        new_content += line
      end
    end
  end
  new_content
end

def process(file)
  dir = File.dirname(file)
  kconfig_file = File.basename(file)
  rewrite(dir, kconfig_file)
end

def uniq_abs_path(paths)
  paths.map { |path| File.realpath(path) }.uniq
end

kconfig_files = uniq_abs_path(ARGV)
kconfig_files.each do |kconfig_file|
  old_kconfig_file = "#{kconfig_file}.old"
  if File.exist?(kconfig_file) && File.file?(kconfig_file)
    rewrited = process(kconfig_file)
    FileUtils.mv kconfig_file, old_kconfig_file
    File.open(kconfig_file, "w").write(rewrited)
  end
end
