# frozen_string_literal: true

require_relative "spec_helper"

VALID_THREAD_SAFE_VALUES = [true, false, "N/A"].freeze

# rubocop:disable Metrics/BlockLength
metadata_array.each do |m|
  RSpec.describe "metadata #{m}" do
    it "has components" do
      expect(m.components?).to be true
    end

    it "has one or more of components" do
      expect(m.components.length).to be >= 1
    end

    m.components.each do |c|
      describe "component #{c}" do
        subject { c }
        it "does not raise error" do
          expect { subject }.not_to raise_error
        end

        it "has name" do
          expect(subject.name?).to be true
        end

        it "has String name" do
          expect(subject.name).to be_kind_of(String)
        end

        it "has non-empty name" do
          expect(subject.name).not_to be_empty
        end

        it "has description" do
          expect(subject.description?).to be true
        end

        it "has String description" do
          expect(subject.description).to be_kind_of(String)
        end

        it "has a primary group" do
          expect(subject.group?).to be true
        end

        it "has a valid primary group" do
          expect { subject.group }.not_to raise_error
        end

        context "when it has one or more groups" do
          it "has zero or more of groups" do
            skip "it has no groups" unless subject.groups?
            expect(subject.groups.length).to be >= 0
          end

          it "has valid groups" do
            skip "it has no groups" unless subject.groups?
            skip "it has zero group" if subject.groups? && subject.groups.empty?
            expect { subject.groups }.not_to raise_error
          end
        end

        context "when it has depends" do
          it "has zero or more of depends" do
            skip "it has no depends" unless subject.depends?
            expect(subject.depends.length).to be >= 0
          end

          it "has valid depends" do
            skip "it has no depends" unless subject.depends?
            skip "it has zero depends" if subject.depends? && subject.depends.empty?
            expect { subject.depends }.not_to raise_error
          end
        end

        it "has thread_safe" do
          expect(subject.thread_safe?).to be true
        end

        it "has valid values of thread_safe" do
          expect(VALID_THREAD_SAFE_VALUES).to include subject.thread_safe
        end

        it "has targets" do
          expect(subject.targets?).to be true
        end

        it "has valid targets" do
          expect { subject.targets }.not_to raise_error
        end

        it "has licenses" do
          expect(subject.licenses?).to be true
        end

        it "has valid licenses" do
          expect { subject.licenses }.not_to raise_error
        end

        it "has one or more of licenses" do
          expect(subject.licenses.length).to be >= 1
        end

        it "has copyrights" do
          expect(subject.copyrights?).to be true
        end

        it "has valid copyrights" do
          expect { subject.copyrights }.not_to raise_error
        end

        it "has one or more of copyrights" do
          expect(subject.copyrights.length).to be >= 1
        end

        it "has Person in all copyrights" do
          subject.copyrights.each do |copyright|
            expect(copyright.name).to be_a Person
          end
        end
      end
    end
  end
end
# rubocop:enable Metrics/BlockLength
