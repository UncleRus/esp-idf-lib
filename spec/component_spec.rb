# frozen_string_literal: true

require_relative "spec_helper"

# rubocop:disable Metrics/BlockLength
components.each do |c|
  RSpec.describe c.to_s do
    subject { c }

    it "has .eil.yml" do
      file = File.join(subject.path, ".eil.yml")
      expect(File.exist?(file)).to be true
    end

    describe "its .eil.yml file" do
      subject { c.metadata }

      it "is valid YAML" do
        expect { subject }.not_to raise_error
      end

      it "has `components` as a top-level key" do
        expect(subject.key?("components")).to be true
      end
    end
  end

  RSpec.describe c.to_s do
    subject { Component.new(c.path) }

    it "does not raise error" do
      expect { subject }.not_to raise_error
    end

    it "has components" do
      expect(subject.components?).to be true
    end

    it "has one or more of components" do
      expect(subject.components.length).to be > 0
    end

    it "has name" do
      expect(subject.name?).to be true
    end

    it "has String name" do
      expect(subject.name).to be_kino_of(String)
    end

    it "has non-empty name" do
      expect(subject.name).not_to be_empty
    end

    it "has description" do
      expect(subject.description?).to be true
    end

    it "has String description" do
      expect(subject.description).to be_kino_of(String)
    end

    it "has a primary group" do
      expect(subject.group?).to be true
    end

    it "has a valid primary group" do
      expect { subject.group }.not_to raise_error
    end
  end
end
# rubocop:enable Metrics/BlockLength
