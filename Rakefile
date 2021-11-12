# frozen_string_literal: true

task default: [:test]

desc "Run all tests"
task test: [:rubocop, :rspec]

desc "Run rubocop"
task :rubocop do
  sh "rubocop"
end

desc "Run rspec"
task :rspec do
  sh "rspec"
end
